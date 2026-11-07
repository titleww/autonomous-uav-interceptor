# Main Thread: Integrates Vision and Flight Control with Guidance State Machine
import os
import time
import yaml
from utils.logger import logger
from ai_vision.camera_csi import get_stream_and_metadata
from ai_vision.overlay import draw_target_overlay
from flight_control.guidance_pid import InterceptPID
from flight_control.guidance_pn import ProportionalNavigation
from flight_control.state_machine import GuidanceStateMachine
from flight_control.mavlink_node import MAVLinkNode

def load_config():
    config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'default.yaml')
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            try:
                return yaml.safe_load(f)
            except Exception as e:
                logger.error(f"Error parsing config file: {e}")
    logger.warning("Config file not found or failed to parse. Using default settings.")
    return {
        'camera': {'resolution': [640, 480], 'fps': 30},
        'flight_controller': {'port': '/dev/serial0', 'baudrate': 115200},
        'guidance': {
            'algorithm': 'HYBRID',
            'pn': {
                'nav_constant': 3.0,
                'closing_velocity': 15.0,
                'max_lateral_velocity': 5.0,
                'max_vertical_velocity': 3.0,
                'filter_alpha': 0.3
            },
            'pid': {
                'kp': 8.0,
                'ki': 0.5,
                'kd': 0.8
            },
            'search': {
                'yaw_rate': 0.5
            },
            'recover': {
                'align_threshold': 0.1,
                'align_duration': 0.5
            }
        }
    }

def main():
    logger.info("Initializing Autonomous-UAV-Interceptor...")
    
    # 1. Load config
    config = load_config()
    res = config['camera']['resolution']
    res_width = res[0]
    
    # 2. Initialize Flight Controller
    fc_config = config['flight_controller']
    try:
        fc = MAVLinkNode(fc_config['port'], fc_config['baudrate'])
        logger.info(f"Connected to F405 at {fc_config['port']} (Baudrate: {fc_config['baudrate']})")
    except Exception as e:
        logger.error(f"Failed to connect to F405: {e}. Running in SITL/Mock mode.")
        fc = None
        
    # 3. Initialize Guidance Controllers
    guid_config = config['guidance']
    algorithm = guid_config.get('algorithm', 'HYBRID').upper()
    logger.info(f"Guidance mode configured to: {algorithm}")
    
    # Instantiate controllers
    pn_params = guid_config['pn']
    pn_guidance = ProportionalNavigation(
        nav_constant=pn_params['nav_constant'],
        closing_velocity=pn_params['closing_velocity'],
        resolution=res,
        filter_alpha=pn_params['filter_alpha']
    )
    
    pid_params = guid_config['pid']
    pid_guidance = InterceptPID(
        kp=pid_params['kp'],
        ki=pid_params['ki'],
        kd=pid_params['kd'],
        output_limits=(-pn_params['max_lateral_velocity'], pn_params['max_lateral_velocity'])
    )
    
    # Search & Recovery Params
    search_params = guid_config.get('search', {'yaw_rate': 0.5})
    recover_params = guid_config.get('recover', {'align_threshold': 0.1, 'align_duration': 0.5})
    
    # Initialize Guidance State Machine
    state_machine = GuidanceStateMachine(
        search_yaw_rate=search_params['yaw_rate'],
        align_threshold=recover_params['align_threshold'],
        align_duration=recover_params['align_duration']
    )
    
    logger.info(f"Initial State: {state_machine.state}")
    prev_time = time.time()
    
    try:
        while True:
            current_time = time.time()
            dt = current_time - prev_time
            if dt <= 0:
                dt = 0.001
                
            # 1. Fetch camera frame and Bounding Box metadata from AI camera
            stream_data = get_stream_and_metadata()
            if stream_data is not None:
                frame, target_bbox = stream_data
            else:
                frame, target_bbox = None, None
                
            # 2. Draw FPV overlay
            if frame is not None:
                draw_target_overlay(frame, target_bbox)
                
            # Initialize command variables
            vx, vy, vz = 0.0, 0.0, 0.0
            yaw_rate_cmd = 0.0
            
            # 3. State Machine Guidance Calculation
            if algorithm == "HYBRID":
                # Update state machine and get state cues
                state, suggested_vx, err_x, suggested_yaw = state_machine.update(target_bbox, current_time)
                
                if state == "SEARCH":
                    vx, vy, vz = 0.0, 0.0, 0.0
                    yaw_rate_cmd = suggested_yaw
                    
                elif state == "PID_RECOVER":
                    # Use PID to align horizontal error
                    vy = pid_guidance.calculate(err_x, current_time=current_time)
                    vx = suggested_vx
                    vz = 0.0
                    yaw_rate_cmd = 0.0
                    
                elif state == "PN_ATTACK":
                    # Run PN for predictive intercept
                    vy, vz = pn_guidance.update_velocities(
                        target_bbox=target_bbox,
                        dt=dt,
                        max_lat_vel=pn_params['max_lateral_velocity'],
                        max_vert_vel=pn_params['max_vertical_velocity'],
                        current_time=current_time
                    )
                    vx = suggested_vx
                    yaw_rate_cmd = 0.0
                    
            else:
                # Fallback to single algorithm mode
                if target_bbox is None:
                    vx, vy, vz = 0.0, 0.0, 0.0
                    yaw_rate_cmd = search_params['yaw_rate']
                else:
                    yaw_rate_cmd = 0.0
                    if algorithm == "PN":
                        vy, vz = pn_guidance.update_velocities(
                            target_bbox=target_bbox,
                            dt=dt,
                            max_lat_vel=pn_params['max_lateral_velocity'],
                            max_vert_vel=pn_params['max_vertical_velocity'],
                            current_time=current_time
                        )
                        vx = pn_params['closing_velocity']
                    else: # PID
                        x_center = target_bbox[0]
                        err_x = (x_center - (res_width / 2.0)) / (res_width / 2.0)
                        vy = pid_guidance.calculate(err_x, current_time=current_time)
                        vx = 5.0
                        vz = 0.0
            
            # 4. Transmit commands to F405 flight controller via MAVLink
            if fc is not None:
                fc.send_velocity_command(vx, vy, vz, yaw_rate=yaw_rate_cmd)
                
            prev_time = current_time
            time.sleep(0.02) # ~50Hz loop
            
    except KeyboardInterrupt:
        logger.info("Shutting down safely.")

if __name__ == "__main__":
    main()
