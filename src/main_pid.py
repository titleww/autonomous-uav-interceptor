# Main Thread for PID-Only Guidance Control (No State Machine / No PN)
import os
import time
import yaml
from utils.logger import logger
from ai_vision.camera_csi import get_stream_and_metadata
from ai_vision.overlay import draw_target_overlay
from flight_control.guidance_pid import InterceptPID
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
            'pid': {
                'kp': 8.0,
                'ki': 0.5,
                'kd': 0.8
            },
            'search': {
                'yaw_rate': 0.5
            }
        }
    }

def main():
    logger.info("Initializing Autonomous-UAV-Interceptor (PID-ONLY Mode)...")
    
    # 1. Load config
    config = load_config()
    res = config['camera']['resolution']
    res_width = res[0]
    
    # 2. Initialize Flight Controller Connection
    fc_config = config['flight_controller']
    try:
        fc = MAVLinkNode(fc_config['port'], fc_config['baudrate'])
        logger.info(f"Connected to F405 at {fc_config['port']} (Baudrate: {fc_config['baudrate']})")
    except Exception as e:
        logger.error(f"Failed to connect to F405: {e}. Running in SITL/Mock mode.")
        fc = None
        
    # 3. Initialize PID Controller only
    guid_config = config['guidance']
    pid_params = guid_config['pid']
    
    # Max lateral speed limit is set to 5.0 m/s
    max_lateral_speed = 5.0
    pid_guidance = InterceptPID(
        kp=pid_params['kp'],
        ki=pid_params['ki'],
        kd=pid_params['kd'],
        output_limits=(-max_lateral_speed, max_lateral_speed)
    )
    
    search_yaw_rate = guid_config.get('search', {'yaw_rate': 0.5})['yaw_rate']
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
                
            # 3. PID Guidance Calculation
            vx, vy, vz = 0.0, 0.0, 0.0
            yaw_rate_cmd = 0.0
            
            if target_bbox is None:
                # Target is lost -> rotate in place to search
                vx, vy, vz = 0.0, 0.0, 0.0
                yaw_rate_cmd = search_yaw_rate
                pid_guidance.reset() # Reset PID history when target is lost
            else:
                # Target detected -> track and chase target
                x_center = target_bbox[0]
                # Normalize error from center to [-1.0, 1.0] range
                err_x = (x_center - (res_width / 2.0)) / (res_width / 2.0)
                
                # Calculate required lateral velocity using PID
                vy = pid_guidance.calculate(err_x, current_time=current_time)
                
                # Constant forward velocity of 5.0 m/s for tracking/approach
                vx = 5.0
                vz = 0.0
                yaw_rate_cmd = 0.0
                
            # 4. Transmit commands to F405 flight controller via MAVLink
            if fc is not None:
                fc.send_velocity_command(vx, vy, vz, yaw_rate=yaw_rate_cmd)
                
            prev_time = current_time
            time.sleep(0.02) # ~50Hz loop
            
    except KeyboardInterrupt:
        logger.info("PID-ONLY controller shutting down safely.")

if __name__ == "__main__":
    main()
