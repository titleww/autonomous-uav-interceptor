# Proportional Navigation (PN) guidance algorithm for target interception
import math
import time

class ProportionalNavigation:
    def __init__(self, nav_constant=3.0, closing_velocity=15.0, fov_h=78.0, fov_v=60.0, resolution=(640, 480), filter_alpha=0.3):
        """
        nav_constant (N): Navigation constant, typically 3.0 to 5.0.
        closing_velocity (Vc): Estimated closing speed in m/s.
        fov_h: Horizontal Field of View of camera in degrees.
        fov_v: Vertical Field of View of camera in degrees.
        resolution: Camera resolution (width, height).
        filter_alpha: Low-pass filter coefficient for smoothing Line-of-Sight rates.
        """
        self.N = nav_constant
        self.Vc = closing_velocity
        self.width = resolution[0]
        self.height = resolution[1]
        self.alpha = filter_alpha
        
        # Camera FOV in radians
        self.fov_h_rad = math.radians(fov_h)
        self.fov_v_rad = math.radians(fov_v)
        
        # Previous Line-of-Sight (LOS) angles in radians
        self.prev_los_yaw = None
        self.prev_los_pitch = None
        self.prev_time = None
        
        # Filtered Line-of-Sight rates (rad/s)
        self.filtered_los_rate_yaw = 0.0
        self.filtered_los_rate_pitch = 0.0
        
        # Integrated velocity commands (starting at 0)
        self.v_lateral = 0.0
        self.v_vertical = 0.0

    def calculate_commands(self, target_bbox, current_time=None):
        """
        Calculates the lateral and vertical velocity adjustments using PN law.
        
        target_bbox: Bounding box data [x_center, y_center, width, height]
                     where x_center and y_center are in absolute pixel coordinates.
        current_time: timestamp in seconds (default uses time.time())
        
        Returns:
            ay (float): Commanded horizontal acceleration (m/s^2)
            az (float): Commanded vertical acceleration (m/s^2)
        """
        if current_time is None:
            current_time = time.time()
            
        if target_bbox is None:
            # If target is lost, return zero acceleration
            return 0.0, 0.0
            
        x_center, y_center, _, _ = target_bbox
        
        # Calculate pixel error from the center of the image
        img_center_x = self.width / 2.0
        img_center_y = self.height / 2.0
        
        err_x = x_center - img_center_x
        err_y = y_center - img_center_y
        
        # Convert pixel errors to Line-of-Sight (LOS) angles relative to camera optical axis
        # err_x > 0 means target is to the right -> positive yaw angle
        # err_y > 0 means target is down in image coordinates -> negative pitch angle (standard body frame: Z is down or up depending on conventions. Let's make it standard: err_y > 0 -> target below centerline -> negative vertical velocity/pitch)
        los_yaw = (err_x / img_center_x) * (self.fov_h_rad / 2.0)
        los_pitch = -(err_y / img_center_y) * (self.fov_v_rad / 2.0)
        
        if self.prev_time is None or self.prev_los_yaw is None:
            # First frame, initialize and return zero acceleration
            self.prev_los_yaw = los_yaw
            self.prev_los_pitch = los_pitch
            self.prev_time = current_time
            return 0.0, 0.0
            
        dt = current_time - self.prev_time
        if dt <= 0.0:
            dt = 0.001 # Prevent division by zero
            
        # Calculate raw Line-of-Sight (LOS) rates (rad/s)
        raw_los_rate_yaw = (los_yaw - self.prev_los_yaw) / dt
        raw_los_rate_pitch = (los_pitch - self.prev_los_pitch) / dt
        
        # Apply low-pass filter to reject camera detection noise / jitter
        self.filtered_los_rate_yaw = (self.alpha * raw_los_rate_yaw) + ((1.0 - self.alpha) * self.filtered_los_rate_yaw)
        self.filtered_los_rate_pitch = (self.alpha * raw_los_rate_pitch) + ((1.0 - self.alpha) * self.filtered_los_rate_pitch)
        
        # Calculate Commanded Lateral Accelerations: a_c = N * V_c * d_lambda
        # ay: horizontal acceleration (m/s^2)
        # az: vertical acceleration (m/s^2)
        ay = self.N * self.Vc * self.filtered_los_rate_yaw
        az = self.N * self.Vc * self.filtered_los_rate_pitch
        
        # Save state for the next calculation
        self.prev_los_yaw = los_yaw
        self.prev_los_pitch = los_pitch
        self.prev_time = current_time
        
        return ay, az

    def update_velocities(self, target_bbox, dt, max_lat_vel=5.0, max_vert_vel=3.0, current_time=None):
        """
        Integrates the acceleration commands to output velocity commands (Vy, Vz).
        
        dt: time step in seconds
        max_lat_vel: safety limit on lateral speed (m/s)
        max_vert_vel: safety limit on vertical speed (m/s)
        
        Returns:
            v_y (float): Commanded horizontal speed (m/s)
            v_z (float): Commanded vertical speed (m/s)
        """
        ay, az = self.calculate_commands(target_bbox, current_time)
        
        if target_bbox is None:
            # Decay velocities slowly if target is lost, to avoid sudden erratic motion
            self.v_lateral *= 0.9
            self.v_vertical *= 0.9
            return self.v_lateral, self.v_vertical
            
        # Integrate acceleration commands
        self.v_lateral += ay * dt
        self.v_vertical += az * dt
        
        # Clip command velocities to physical safety limits of the drone
        self.v_lateral = max(-max_lat_vel, min(self.v_lateral, max_lat_vel))
        self.v_vertical = max(-max_vert_vel, min(self.v_vertical, max_vert_vel))
        
        return self.v_lateral, self.v_vertical
