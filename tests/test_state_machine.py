import unittest
from src.flight_control.state_machine import GuidanceStateMachine

class TestGuidanceStateMachine(unittest.TestCase):
    def setUp(self):
        # Initialize with yaw_rate=0.5, threshold=0.1, duration=0.5 seconds
        self.sm = GuidanceStateMachine(search_yaw_rate=0.5, align_threshold=0.1, align_duration=0.5)

    def test_initial_state_search(self):
        # Initial state should be SEARCH
        self.assertEqual(self.sm.state, "SEARCH")
        
        # When target is None, should return SEARCH state and search yaw rate
        state, vx, err_x, yaw_rate = self.sm.update(None, current_time=0.0)
        self.assertEqual(state, "SEARCH")
        self.assertEqual(vx, 0.0)
        self.assertEqual(yaw_rate, 0.5)

    def test_target_detected_transitions_to_recover(self):
        # Target found -> State goes to PID_RECOVER
        state, vx, err_x, yaw_rate = self.sm.update([400, 240, 10, 10], current_time=0.0)
        self.assertEqual(self.sm.state, "PID_RECOVER")
        self.assertEqual(state, "PID_RECOVER")
        self.assertEqual(vx, 3.0)  # Slow approach speed
        self.assertEqual(yaw_rate, 0.0)

    def test_alignment_timer_and_pn_transition(self):
        # 1. Detect target (starts in PID_RECOVER)
        # target_x = 330, width = 640. Normalized err_x = (330 - 320) / 320 = 10 / 320 = 0.03125 (< 0.1 threshold)
        self.sm.update([330, 240, 10, 10], current_time=0.0)
        self.assertEqual(self.sm.state, "PID_RECOVER")
        self.assertIsNotNone(self.sm.align_start_time)
        
        # 2. Keep aligned for 0.3s (less than 0.5s duration) -> stays in PID_RECOVER
        state, vx, err_x, yaw_rate = self.sm.update([330, 240, 10, 10], current_time=0.3)
        self.assertEqual(self.sm.state, "PID_RECOVER")
        self.assertEqual(vx, 3.0)
        
        # 3. Reach 0.5s duration -> transitions to PN_ATTACK
        state, vx, err_x, yaw_rate = self.sm.update([330, 240, 10, 10], current_time=0.5)
        self.assertEqual(self.sm.state, "PN_ATTACK")
        self.assertEqual(state, "PN_ATTACK")
        self.assertEqual(vx, 15.0)  # Full PN attack speed
        self.assertEqual(yaw_rate, 0.0)

    def test_alignment_reset_if_error_exceeds_threshold(self):
        # 1. Detect target aligned
        self.sm.update([330, 240, 10, 10], current_time=0.0)
        self.assertEqual(self.sm.state, "PID_RECOVER")
        self.assertIsNotNone(self.sm.align_start_time)
        
        # 2. target moves off-center (err_x > 0.1 threshold) -> resets align timer
        # target_x = 400. err_x = (400 - 320) / 320 = 80 / 320 = 0.25 (> 0.1)
        self.sm.update([400, 240, 10, 10], current_time=0.3)
        self.assertEqual(self.sm.state, "PID_RECOVER")
        self.assertIsNone(self.sm.align_start_time)

    def test_target_lost_transitions_to_search(self):
        # 1. Start in PID_RECOVER
        self.sm.update([330, 240, 10, 10], current_time=0.0)
        
        # 2. Target aligned -> moves to PN_ATTACK
        self.sm.update([330, 240, 10, 10], current_time=0.5)
        self.assertEqual(self.sm.state, "PN_ATTACK")
        
        # 3. Target lost -> transitions back to SEARCH
        state, vx, err_x, yaw_rate = self.sm.update(None, current_time=0.6)
        self.assertEqual(self.sm.state, "SEARCH")
        self.assertEqual(state, "SEARCH")
        self.assertEqual(vx, 0.0)
        self.assertEqual(yaw_rate, 0.5)

if __name__ == '__main__':
    unittest.main()
