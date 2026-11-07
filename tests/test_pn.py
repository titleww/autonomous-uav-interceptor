import unittest
import time
from src.flight_control.guidance_pn import ProportionalNavigation

class TestProportionalNavigation(unittest.TestCase):
    def setUp(self):
        # Initialize PN with N=3.0, Vc=15.0, FOV=78x60, Resolution=640x480, alpha=1.0 (no filtering for raw test)
        self.pn = ProportionalNavigation(
            nav_constant=3.0,
            closing_velocity=15.0,
            fov_h=78.0,
            fov_v=60.0,
            resolution=(640, 480),
            filter_alpha=1.0
        )

    def test_target_centered(self):
        # 1. First step to initialize
        ay, az = self.pn.calculate_commands([320, 240, 10, 10], current_time=0.0)
        self.assertEqual(ay, 0.0)
        self.assertEqual(az, 0.0)

        # 2. Target remains centered
        ay, az = self.pn.calculate_commands([320, 240, 10, 10], current_time=0.1)
        self.assertEqual(ay, 0.0)
        self.assertEqual(az, 0.0)

    def test_target_moving_right(self):
        # 1. Initialize with target slightly right of center
        self.pn.calculate_commands([330, 240, 10, 10], current_time=0.0)
        
        # 2. Target moves further right (from x=330 to x=350)
        # This means positive horizontal error rate (LOS yaw is rotating right)
        ay, az = self.pn.calculate_commands([350, 240, 10, 10], current_time=0.1)
        
        # Acceleration in yaw (horizontal) should be positive
        self.assertGreater(ay, 0.0)
        # Vertical acceleration should be zero since target remains at y=240
        self.assertEqual(az, 0.0)

    def test_target_moving_down(self):
        # 1. Initialize
        self.pn.calculate_commands([320, 230, 10, 10], current_time=0.0)
        
        # 2. Target moves down (from y=230 to y=250 in image coordinates, i.e., moving below center)
        # In image coordinates, moving down increases y value, which corresponds to pitch going negative.
        ay, az = self.pn.calculate_commands([320, 250, 10, 10], current_time=0.1)
        
        # Horizontal acceleration is zero
        self.assertEqual(ay, 0.0)
        # Vertical acceleration should be negative (commanding down)
        self.assertLess(az, 0.0)

    def test_velocity_integration_and_clipping(self):
        # Test velocity calculation and limit clipping
        # 1. Initialize
        self.pn.update_velocities([320, 240, 10, 10], dt=0.1, current_time=0.0)
        
        # 2. Move right significantly to generate high acceleration command
        vy, vz = self.pn.update_velocities([450, 240, 10, 10], dt=0.1, max_lat_vel=5.0, max_vert_vel=3.0, current_time=0.1)
        self.assertGreater(vy, 0.0)
        self.assertEqual(vz, 0.0)

        # 3. Simulate continuous high acceleration to trigger clipping limit
        for i in range(10):
            vy, vz = self.pn.update_velocities([600, 240, 10, 10], dt=0.1, max_lat_vel=5.0, max_vert_vel=3.0, current_time=0.2 + i*0.1)
            
        # Velocities must be capped at max_lat_vel and max_vert_vel
        self.assertEqual(vy, 5.0)
        self.assertEqual(vz, 0.0)

    def test_target_lost_decay(self):
        # Target lost should slowly decay velocity command to zero to prevent radical behavior
        self.pn.v_lateral = 4.0
        self.pn.v_vertical = -2.0
        
        vy, vz = self.pn.update_velocities(None, dt=0.1)
        self.assertEqual(vy, 4.0 * 0.9)
        self.assertEqual(vz, -2.0 * 0.9)

if __name__ == '__main__':
    unittest.main()
