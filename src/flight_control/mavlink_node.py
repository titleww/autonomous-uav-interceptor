# Handles UART communication with F405 via MAVLink
from pymavlink import mavutil
import time

class MAVLinkNode:
    def __init__(self, port, baudrate):
        # Establish serial connection
        self.connection = mavutil.mavlink_connection(port, baud=baudrate)
        
    def send_velocity_command(self, vx, vy, vz, yaw_rate=0.0):
        """
        Sends velocity and yaw rate commands to the flight controller (F405).
        
        vx: forward velocity (m/s)
        vy: lateral velocity (m/s)
        vz: vertical velocity (m/s, positive is down in NED)
        yaw_rate: yaw rotation speed (rad/s)
        """
        # Coordinate frame: MAV_FRAME_BODY_NED (relative to drone body orientation)
        # Type mask: 967 (binary 0b01111000111)
        # Ignores: Position X, Y, Z (bits 1,2,3), Acceleration X, Y, Z (bits 7,8,9), and Yaw angle (bit 10)
        # Uses: Velocity X, Y, Z (bits 4,5,6) and Yaw Rate (bit 11)
        type_mask = 967
        
        try:
            self.connection.mav.set_position_target_local_ned_send(
                int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
                1,  # target_system (sysid)
                1,  # target_component (compid)
                mavutil.mavlink.MAV_FRAME_BODY_NED,  # coordinate_frame
                type_mask,
                0.0, 0.0, 0.0,  # position (x, y, z) - IGNORED
                vx, vy, vz,     # velocity (vx, vy, vz) - USED
                0.0, 0.0, 0.0,  # acceleration (afx, afy, afz) - IGNORED
                0.0,            # yaw - IGNORED
                yaw_rate        # yaw_rate (rad/s) - USED
            )
        except Exception as e:
            # Catch serial errors gracefully
            pass
