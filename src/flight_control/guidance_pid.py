# PID Algorithm for intercept trajectory
import time

class InterceptPID:
    def __init__(self, kp, ki, kd, output_limits=(-5.0, 5.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        self.min_output = output_limits[0]
        self.max_output = output_limits[1]

    def calculate(self, target_error, current_time=None):
        """
        Calculates the PID control output based on current error.
        
        target_error: difference between target center and image center
        current_time: timestamp in seconds
        
        Returns:
            output (float): commanded speed/adjustment
        """
        if current_time is None:
            current_time = time.time()
            
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = target_error
            return 0.0
            
        dt = current_time - self.prev_time
        if dt <= 0.0:
            dt = 0.001  # Prevent division by zero
            
        # 1. Proportional Term (reactive to current error)
        p_out = self.kp * target_error
        
        # 2. Integral Term (reactive to accumulated past errors, with Anti-Windup)
        self.integral += target_error * dt
        # Anti-windup: limit the integral to prevent runaway overshoot
        ki_divisor = self.ki if self.ki != 0.0 else 1.0
        max_integral_acc = self.max_output / ki_divisor
        self.integral = max(-max_integral_acc, min(self.integral, max_integral_acc))
        i_out = self.ki * self.integral
        
        # 3. Derivative Term (predictive based on error rate of change)
        derivative = (target_error - self.prev_error) / dt
        d_out = self.kd * derivative
        
        # 4. Total Output
        output = p_out + i_out + d_out
        
        # 5. Output Limiting
        output = max(self.min_output, min(output, self.max_output))
        
        # Save state for the next calculation loop
        self.prev_error = target_error
        self.prev_time = current_time
        
        return output

    def reset(self):
        """
        Resets the controller integral and history state.
        """
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
