# Guidance State Machine for Interceptor flight modes
import time

class GuidanceStateMachine:
    def __init__(self, search_yaw_rate=0.5, align_threshold=0.1, align_duration=0.5):
        """
        Manages the state transitions of the interceptor.
        
        search_yaw_rate: rotation speed (rad/s) when searching
        align_threshold: normalized error threshold to consider target centered
        align_duration: duration (seconds) target must remain aligned before striking
        """
        self.state = "SEARCH"
        self.search_yaw_rate = search_yaw_rate
        self.align_threshold = align_threshold
        self.align_duration = align_duration
        self.align_start_time = None

    def update(self, target_bbox, current_time=None):
        """
        Updates the state machine based on target bounding box and timestamp.
        
        target_bbox: Bounding box [x_center, y_center, w, h] or None
        current_time: timestamp in seconds
        
        Returns:
            state (str): current active state
            vx (float): suggested forward velocity
            err_x (float): normalized error (used if in PID_RECOVER)
            yaw_rate (float): suggested yaw rate command
        """
        if current_time is None:
            current_time = time.time()
            
        # 1. External Transitions
        if target_bbox is None:
            if self.state != "SEARCH":
                self.state = "SEARCH"
                self.align_start_time = None
        else:
            if self.state == "SEARCH":
                self.state = "PID_RECOVER"
                self.align_start_time = None

        # 2. Internal Transitions
        if self.state == "PID_RECOVER":
            x_center = target_bbox[0]
            err_x = (x_center - 320.0) / 320.0
            
            # Check alignment threshold
            if abs(err_x) < self.align_threshold:
                if self.align_start_time is None:
                    self.align_start_time = current_time
                elif current_time - self.align_start_time >= self.align_duration:
                    self.state = "PN_ATTACK"
                    self.align_start_time = None
            else:
                self.align_start_time = None

        # 3. Execute Active State
        if self.state == "SEARCH":
            # Rotate in place to scan
            return "SEARCH", 0.0, 0.0, self.search_yaw_rate
            
        elif self.state == "PID_RECOVER":
            x_center = target_bbox[0]
            err_x = (x_center - 320.0) / 320.0
            # Slow forward approach velocity (3.0 m/s) to allow stable alignment turn
            return "PID_RECOVER", 3.0, err_x, 0.0
            
        elif self.state == "PN_ATTACK":
            # Fast PN interception speed (15.0 m/s)
            return "PN_ATTACK", 15.0, 0.0, 0.0

    def reset(self):
        """
        Resets the state machine to initial SEARCH state.
        """
        self.state = "SEARCH"
        self.align_start_time = None
