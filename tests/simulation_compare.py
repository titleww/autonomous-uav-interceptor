# Simulation comparison between PID and PN guidance algorithms
import sys
import os
import math

# Add src to python path to import modules
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'src'))

from flight_control.guidance_pid import InterceptPID
from flight_control.guidance_pn import ProportionalNavigation

def run_simulation(algo_type, scenario_type, max_steps=500, dt=0.02):
    """
    Runs a 2D physics simulation of target interception.
    
    algo_type: "PID" or "PN"
    scenario_type: "stationary" or "cross-flying"
    
    Returns:
        trajectory: list of tuples (time, x_drone, y_drone, x_target, y_target, lateral_speed, lateral_accel)
        success: bool (true if intercepted within 1.0m)
        miss_distance: float (minimum distance to target)
    """
    # 1. Initialize Drone and Target positions
    # Drone starts at (0, 0) flying in X direction at 15 m/s
    x_drone = 0.0
    y_drone = 0.0
    vx = 15.0
    vy = 0.0
    
    # Target settings based on scenario
    if scenario_type == "stationary":
        # Target stands still at (100, 20)
        x_target = 100.0
        y_target = 20.0
        target_vx = 0.0
        target_vy = 0.0
    elif scenario_type == "cross-flying":
        # Target starts at (100, -20) moving perpendicular at 5 m/s
        x_target = 100.0
        y_target = -20.0
        target_vx = 0.0
        target_vy = 5.0
    else:
        raise ValueError("Unknown scenario type")

    # Camera constants
    width = 640
    height = 480
    fov_h_rad = math.radians(78.0)
    # Focal length in pixels
    focal_pixels = (width / 2.0) / math.tan(fov_h_rad / 2.0)

    # 2. Initialize Controllers
    if algo_type == "PID":
        # PID tuned for target tracking (error range [-1, 1], outputs lateral speed Vy in [-5, 5])
        controller = InterceptPID(kp=8.0, ki=0.5, kd=0.8, output_limits=(-5.0, 5.0))
    elif algo_type == "PN":
        # Proportional Navigation (N=3, Vc=15, alpha=0.5 for smoothing)
        controller = ProportionalNavigation(
            nav_constant=3.0,
            closing_velocity=15.0,
            fov_h=78.0,
            fov_v=60.0,
            resolution=(width, height),
            filter_alpha=0.5
        )
    else:
        raise ValueError("Unknown algorithm type")

    trajectory = []
    min_dist = float('inf')
    success = False
    
    # Simulation loop
    for step in range(max_steps):
        t = step * dt
        
        # Calculate distance
        dx = x_target - x_drone
        dy = y_target - y_drone
        dist = math.sqrt(dx*dx + dy*dy)
        
        if dist < min_dist:
            min_dist = dist
            
        # Check if we have passed the target (interceptor goes past target's X position)
        if dx < 0.0:
            break
            
        # Target detection simulated through camera frame mapping
        # Relative angle to target in drone body frame
        angle_to_target = math.atan2(dy, dx)
        
        # If target is outside the camera's FOV (e.g. angle > fov/2), we cannot see it
        if abs(angle_to_target) > (fov_h_rad / 2.0):
            target_bbox = None
        else:
            # Map angle to pixel coordinate x_pixel
            # Directly proportional: center is 320, leftmost is 0 (-fov_h/2), rightmost is 640 (+fov_h/2)
            x_pixel = (width / 2.0) + (angle_to_target / (fov_h_rad / 2.0)) * (width / 2.0)
            
            # Map distance to bounding box width
            bbox_width = (1.0 / dx) * focal_pixels if dx > 0.1 else width
            
            # Bounding box [x_center, y_center, w, h]
            target_bbox = [x_pixel, 240, bbox_width, bbox_width]
            
        # Calculate control commands
        prev_vy = vy
        if algo_type == "PID":
            if target_bbox is not None:
                # Normalize pixel error to [-1.0, 1.0] range
                err_x = (target_bbox[0] - (width / 2.0)) / (width / 2.0)
                vy = controller.calculate(err_x, current_time=t)
            else:
                vy *= 0.9  # Decay
        elif algo_type == "PN":
            # Update lateral velocity Vy using PN integration
            vy, _ = controller.update_velocities(
                target_bbox=target_bbox,
                dt=dt,
                max_lat_vel=5.0,
                max_vert_vel=3.0,
                current_time=t
            )
            
        # Calculate lateral acceleration
        ay = (vy - prev_vy) / dt
        
        # Record state
        trajectory.append((t, x_drone, y_drone, x_target, y_target, vy, ay))
        
        # Update positions
        x_drone += vx * dt
        y_drone += vy * dt
        
        x_target += target_vx * dt
        y_target += target_vy * dt

    success = (min_dist <= 1.5) # Intercept successful if within 1.5m
    return trajectory, success, min_dist

def print_text_graph(pid_traj, pn_traj, scenario_name):
    """
    Renders a simple ASCII comparison graph of the trajectories.
    """
    print(f"\n==================================================")
    print(f" Flight Path ASCII Plot: {scenario_name.upper()}")
    print(f"==================================================")
    print(" (Y is horizontal vertical axis, X is horizontal axis pointing right)")
    print(" P = PID Trajectory, N = PN Trajectory, T = Target Trajectory, * = Intercept Point\n")
    
    # Setup grid
    cols = 65
    rows = 15
    grid = [[' ' for _ in range(cols)] for _ in range(rows)]
    
    # Scale simulation coordinates to grid size
    # X goes from 0 to 110. Y goes from -25 to 25.
    def scale(x, y):
        col = int((x / 110.0) * (cols - 1))
        # Y is row: -25 is bottom row, +25 is top row
        row = int(((25.0 - y) / 50.0) * (rows - 1))
        # Clamp bounds
        col = max(0, min(col, cols - 1))
        row = max(0, min(row, rows - 1))
        return col, row

    # Draw grid boundaries/lines
    for c in range(cols):
        grid[rows // 2][c] = '-'
    for r in range(rows):
        grid[r][0] = '|'
        
    # Draw Target path
    for state in pid_traj: # target path is the same for both, we can just trace from pid_traj
        _, _, _, tx, ty, _, _ = state
        tc, tr = scale(tx, ty)
        grid[tr][tc] = 'T'
        
    # Draw PID path
    for state in pid_traj:
        _, px, py, _, _, _, _ = state
        pc, pr = scale(px, py)
        if grid[pr][pc] == 'T':
            grid[pr][pc] = '*'
        else:
            grid[pr][pc] = 'P'
            
    # Draw PN path
    for state in pn_traj:
        _, nx, ny, _, _, _, _ = state
        nc, nr = scale(nx, ny)
        if grid[nr][nc] == 'T':
            grid[nr][nc] = '*'
        elif grid[nr][nc] == 'P':
            grid[nr][nc] = 'X' # Overlap
        else:
            grid[nr][nc] = 'N'
            
    # Print the grid
    for r in grid:
        print(''.join(r))
    print("-" * cols)

def main():
    scenarios = ["stationary", "cross-flying"]
    
    for scenario in scenarios:
        print(f"\nRunning simulation for Scenario: {scenario.upper()}")
        
        # 1. Run PID Simulation
        pid_traj, pid_success, pid_miss = run_simulation("PID", scenario)
        
        # 2. Run PN Simulation
        pn_traj, pn_success, pn_miss = run_simulation("PN", scenario)
        
        # 3. Print Results Summary
        print("\n--- Interception Simulation Summary ---")
        print(f"PID: Success = {pid_success}, Miss Distance = {pid_miss:.4f}m, Flight Time = {pid_traj[-1][0]:.2f}s")
        print(f"PN : Success = {pn_success}, Miss Distance = {pn_miss:.4f}m, Flight Time = {pn_traj[-1][0]:.2f}s")
        
        # 4. Analyze peak lateral acceleration (handling forces)
        pid_peak_accel = max(abs(s[6]) for s in pid_traj)
        pn_peak_accel = max(abs(s[6]) for s in pn_traj)
        
        # Analyze terminal acceleration (last 1 second of flight)
        # 1 second represents 50 steps
        pid_term_steps = pid_traj[-50:] if len(pid_traj) >= 50 else pid_traj
        pn_term_steps = pn_traj[-50:] if len(pn_traj) >= 50 else pn_traj
        pid_term_avg_accel = sum(abs(s[6]) for s in pid_term_steps) / len(pid_term_steps)
        pn_term_avg_accel = sum(abs(s[6]) for s in pn_term_steps) / len(pn_term_steps)
        
        print(f"PID Peak Acceleration: {pid_peak_accel:.2f} m/s^2, Terminal Avg Acceleration: {pid_term_avg_accel:.2f} m/s^2")
        print(f"PN  Peak Acceleration: {pn_peak_accel:.2f} m/s^2, Terminal Avg Acceleration: {pn_term_avg_accel:.2f} m/s^2")
        
        # Render ASCII graph
        print_text_graph(pid_traj, pn_traj, scenario)

if __name__ == "__main__":
    main()
