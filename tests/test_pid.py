import unittest
import time
from src.flight_control.guidance_pid import InterceptPID

class TestPID(unittest.TestCase):
    def test_zero_error(self):
        pid = InterceptPID(kp=1.0, ki=0.0, kd=0.0)
        # Initialize
        pid.calculate(0.0, current_time=0.0)
        # Check calculation
        self.assertEqual(pid.calculate(0.0, current_time=0.1), 0.0)

    def test_proportional_response(self):
        pid = InterceptPID(kp=2.5, ki=0.0, kd=0.0, output_limits=(-10.0, 10.0))
        pid.calculate(0.0, current_time=0.0)
        
        # Positive error yields positive output (Kp * error)
        out = pid.calculate(2.0, current_time=0.1)
        self.assertEqual(out, 5.0)

    def test_integral_accumulation_and_windup(self):
        pid = InterceptPID(kp=0.0, ki=1.0, kd=0.0, output_limits=(-5.0, 5.0))
        # Initialize
        pid.calculate(2.0, current_time=0.0)
        
        # Accumulate error over time steps
        # Step 1: dt = 0.5s, error = 2.0 -> integral = 1.0 -> i_out = 1.0
        out1 = pid.calculate(2.0, current_time=0.5)
        self.assertEqual(out1, 1.0)
        
        # Step 2: dt = 1.0s, error = 2.0 -> integral = 3.0 -> i_out = 3.0
        out2 = pid.calculate(2.0, current_time=1.5)
        self.assertEqual(out2, 3.0)
        
        # Step 3: dt = 2.0s, error = 2.0 -> integral would be 7.0 but clamped to max_output/ki = 5.0
        out3 = pid.calculate(2.0, current_time=3.5)
        self.assertEqual(out3, 5.0)
        self.assertEqual(pid.integral, 5.0)

    def test_derivative_kick(self):
        # Kd * de/dt
        pid = InterceptPID(kp=0.0, ki=0.0, kd=2.0, output_limits=(-20.0, 20.0))
        # Initialize with error=0
        pid.calculate(0.0, current_time=0.0)
        
        # Error changes from 0 to 4 over 0.5s -> de/dt = 8 -> d_out = 16
        out = pid.calculate(4.0, current_time=0.5)
        self.assertEqual(out, 16.0)

    def test_reset(self):
        pid = InterceptPID(kp=1.0, ki=1.0, kd=1.0)
        pid.calculate(1.0, current_time=0.0)
        pid.calculate(2.0, current_time=0.1)
        
        self.assertIsNotNone(pid.prev_time)
        self.assertNotEqual(pid.integral, 0.0)
        
        # Reset controller
        pid.reset()
        self.assertIsNone(pid.prev_time)
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.prev_error, 0.0)

if __name__ == '__main__':
    unittest.main()
