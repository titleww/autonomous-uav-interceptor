#!/usr/bin/env python3
# MAVLink Motor Testing and Arming Utility for Autonomous-UAV-Interceptor
import sys
import time
import argparse
from pymavlink import mavutil

# CRITICAL SAFETY NOTE:
# ALWAYS REMOVE ALL PROPELLERS FROM THE DRONE BEFORE TESTING MOTORS ON THE BENCH!
# FAILURE TO DO SO CAN RESULT IN SEVERE PHYSICAL INJURY AND HARDWARE DAMAGE.

def wait_for_ack(master, command_id):
    """Waits for COMMAND_ACK message from the flight controller."""
    print("Waiting for command acknowledgment...")
    start_time = time.time()
    while time.time() - start_time < 4.0:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1.0)
        if msg:
            if msg.command == command_id:
                result_map = {
                    0: "ACCEPTED (Success)",
                    1: "TEMPORARILY_REJECTED (Try again, safety switch might be active)",
                    2: "DENIED (FC refused command)",
                    3: "UNSUPPORTED (Command not supported by FC firmware)",
                    4: "FAILED (Execution error)",
                    5: "IN_PROGRESS (Command being executed)"
                }
                result_str = result_map.get(msg.result, f"UNKNOWN (code {msg.result})")
                print(f"-> ACK Received for Command {command_id}: {result_str}")
                return msg.result
    print("-> Timeout: No ACK received from flight controller.")
    return None

def arm_vehicle(master, arm=True):
    """Sends arm (1) or disarm (0) command."""
    cmd = 1 if arm else 0
    cmd_name = "ARM" if arm else "DISARM"
    print(f"Sending {cmd_name} command to flight controller...")
    
    # MAV_CMD_COMPONENT_ARM_DISARM (400)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,      # confirmation
        cmd,    # param1: 1 = arm, 0 = disarm
        0,      # param2: force arm/disarm (21196 to force arm in ArduPilot - DANGEROUS)
        0, 0, 0, 0, 0 # unused params
    )
    return wait_for_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)

def test_motor(master, motor_num, throttle_pct, duration):
    """Sends MAV_CMD_DO_MOTOR_TEST command to spin a single motor."""
    print(f"Sending Motor Test Command: Motor {motor_num} at {throttle_pct}% throttle for {duration} seconds...")
    
    # MAV_CMD_DO_MOTOR_TEST (320)
    # Param 1: Motor instance number (1-indexed)
    # Param 2: Throttle type (0 = percent, 1 = PWM, 2 = pilot thrust)
    # Param 3: Throttle value (percentage 0-100 or PWM 1000-2000)
    # Param 4: Timeout (seconds)
    # Param 5: Motor count (default 1)
    # Param 6: Test type (0 = board order, 1 = vehicle order)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        motor_num,
        0, # Percent throttle type
        throttle_pct,
        duration,
        0,
        0,
        0
    )
    return wait_for_ack(master, mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST)

def main():
    parser = argparse.ArgumentParser(description="MAVLink Arming and Motor Test Utility")
    parser.add_argument('--port', type=str, default='/dev/serial0', help='Serial port (default: /dev/serial0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baudrate (default: 115200)')
    args = parser.parse_args()

    print("================================================================")
    print("         MAVLink Motor & Arming Test Utility (Autonomous UAV Interceptor)")
    print("================================================================")
    print("⚠️  CRITICAL SAFETY WARNING: REMOVE ALL PROPELLERS BEFORE RUNNING!")
    print("================================================================")
    
    print(f"Connecting to Flight Controller on {args.port} at {args.baud} baud...")
    try:
        master = mavutil.mavlink_connection(args.port, baud=args.baud)
    except Exception as e:
        print(f"Connection error: {e}")
        sys.exit(1)

    print("Waiting for heartbeat from FC...")
    master.wait_heartbeat()
    print(f"Connected successfully! Target System: {master.target_system}, Component: {master.target_component}")
    
    # Request telemetry stream to keep connection alive and verify data
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4, # Hz rate
        1  # Start
    )

    while True:
        print("\nAvailable Commands:")
        print("1. ARM   - Arm the drone motors")
        print("2. DISARM - Disarm the drone motors")
        print("3. TEST  - Test a single motor (MAV_CMD_DO_MOTOR_TEST)")
        print("4. EXIT  - Exit the program")
        
        choice = input("Select an option (1-4): ").strip()
        
        if choice == '1':
            confirm = input("Are you SURE all propellers are removed? (yes/no): ").strip().lower()
            if confirm == 'yes':
                arm_value = arm_vehicle(master, arm=True)
                if arm_value != 0:
                    print("Note: If arming was denied, make sure the safety switch is disabled and pre-arm checks pass.")
            else:
                print("Arming cancelled.")
                
        elif choice == '2':
            arm_vehicle(master, arm=False)
            
        elif choice == '3':
            confirm = input("Are you SURE all propellers are removed? (yes/no): ").strip().lower()
            if confirm == 'yes':
                try:
                    motor = int(input("Motor Number (1-4): ").strip())
                    throttle = int(input("Throttle Percent (5-15% recommended for safety): ").strip())
                    sec = float(input("Duration (seconds): ").strip())
                    
                    if not (1 <= motor <= 8):
                        print("Error: Motor number must be between 1 and 8.")
                        continue
                    if not (0 <= throttle <= 30):
                        print("Warning: Throttle limited to maximum 30% for bench testing.")
                        throttle = min(throttle, 30)
                        
                    test_motor(master, motor, throttle, sec)
                except ValueError:
                    print("Error: Invalid inputs.")
            else:
                print("Motor test cancelled.")
                
        elif choice == '4' or choice.lower() == 'exit':
            print("Disarming before exit as safety precaution...")
            arm_vehicle(master, arm=False)
            print("Exiting utility.")
            break
        else:
            print("Invalid selection.")

if __name__ == '__main__':
    main()
