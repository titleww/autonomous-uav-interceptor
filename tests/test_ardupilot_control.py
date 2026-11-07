# Code to test ArduPilot (MAVLink) Offboard Control on Raspberry Pi
import time
import sys
from pymavlink import mavutil

# ⚠️ CRITICAL SAFETY WARNING: REMOVE ALL PROPELLERS BEFORE RUNNING!

def main():
    port = '/dev/serial0'
    baud = 115200
    
    print("================================================================")
    print("      ArduPilot MAVLink Control Test Utility (Autonomous UAV Interceptor)")
    print("================================================================")
    print("⚠️  CRITICAL SAFETY WARNING: REMOVE ALL PROPELLERS BEFORE RUNNING!")
    print("================================================================")
    
    # Confirm propellers are off
    confirm = input("คุณแน่ใจแล้วใช่ไหมว่าได้ 'ถอดใบพัดโดรนออกทั้งหมดแล้ว'? (yes/no): ").strip().lower()
    if confirm != 'yes':
        print("ยกเลิกการทำงานเพื่อความปลอดภัย")
        return
        
    print(f"\nกำลังเปิดพอร์ตซีเรียล {port} ({baud} baud)...")
    try:
        master = mavutil.mavlink_connection(port, baud=baud)
    except Exception as e:
        print(f"❌ ไม่สามารถเปิดพอร์ตได้: {e}")
        return
        
    print("1/5 รอรับสัญญาณ Heartbeat จาก ArduPilot...")
    heartbeat = master.wait_heartbeat(timeout=15.0)
    if heartbeat is not None:
        print(f"✅ พบ ArduPilot (System ID: {master.target_system}, Component ID: {master.target_component})")
    else:
        print("❌ ไม่พบการเชื่อมต่อ (Timeout 15 วินาที) โปรดตรวจสอบสายสัญญาณและระดับบอร์ด")
        return

    # 2. Switch to GUIDED mode
    print("\n2/5 กำลังสลับโหมดการบินเป็น GUIDED...")
    # Get mode ID for GUIDED
    mode = 'GUIDED'
    if mode not in master.mode_mapping():
        print(f"❌ ไม่พบโหมด {mode} ในโหมดการบินที่บอร์ดรองรับ")
        return
    mode_id = master.mode_mapping()[mode]
    
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    
    # Verify mode change
    mode_changed = False
    start_time = time.time()
    while time.time() - start_time < 5.0:
        message = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if message:
            # Check current mode
            custom_mode = message.custom_mode
            # Find the name corresponding to this mode_id
            for name, id_val in master.mode_mapping().items():
                if id_val == custom_mode and name == 'GUIDED':
                    mode_changed = True
                    break
            if mode_changed:
                break
                
    if mode_changed:
        print("✅ สลับโหมดเป็น GUIDED สำเร็จ!")
    else:
        print("❌ ไม่สามารถสลับโหมดเป็น GUIDED ได้ (โปรดตรวจสอบ Pre-arm warnings ใน QGroundControl)")
        return

    # 3. Arm motors
    print("\n3/5 กำลังสั่ง Arm มอเตอร์ (ปลดล็อกมอเตอร์)...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # 1 = Arm, 0 = Disarm
        0, 0, 0, 0, 0, 0
    )
    
    # Wait to verify arming state
    armed = False
    start_time = time.time()
    while time.time() - start_time < 5.0:
        message = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if message:
            # Check if ARMED flag is present in base_mode
            if message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                armed = True
                break
                
    if armed:
        print("✅ ปลดล็อกมอเตอร์ (Armed) สำเร็จ! มอเตอร์จะเริ่มหมุนเบาๆ (Idle Spin)")
    else:
        print("❌ ไม่สามารถ Arm มอเตอร์ได้ (โปรดตรวจสอบเหตุผลการล็อกใน QGroundControl)")
        return

    # 4. Send velocity command
    print("\n4/5 กำลังส่งคำสั่งเคลื่อนที่ (หมุนมอเตอร์เร่งความเร็วเล็กน้อย) นาน 4 วินาที...")
    vx = 0.5  # เดินหน้า 0.5 m/s
    vy = 0.0
    vz = 0.0
    yaw_rate = 0.0
    type_mask = 967  # ignore position, acceleration, and absolute yaw
    
    start_time = time.time()
    try:
        while time.time() - start_time < 4.0:
            master.mav.set_position_target_local_ned_send(
                int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                type_mask,
                0.0, 0.0, 0.0,  # position (ignored)
                vx, vy, vz,     # velocity (used)
                0.0, 0.0, 0.0,  # acceleration (ignored)
                0.0,            # yaw (ignored)
                yaw_rate        # yaw_rate (used)
            )
            time.sleep(0.1) # ส่งสัญญาณความถี่ 10Hz
    except KeyboardInterrupt:
        print("\nหยุดคำสั่งเร่งกะทันหัน")

    # 5. Disarm motors
    print("\n5/5 กำลังสั่งล็อกมอเตอร์ (Disarmed)...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,  # 0 = Disarm
        0, 0, 0, 0, 0, 0
    )
    
    # Verify disarming
    disarmed = False
    start_time = time.time()
    while time.time() - start_time < 5.0:
        message = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if message:
            if not (message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                disarmed = True
                break
                
    if disarmed:
        print("✅ ล็อกมอเตอร์ (Disarmed) สำเร็จ!")
    else:
        print("⚠️  ล็อกมอเตอร์ล้มเหลว! กรุณาถอดสายแบตเตอรี่ออกทันที!")

    print("\n================================================================")
    print("🎉 การทดสอบเสร็จสมบูรณ์!")
    print("================================================================")

if __name__ == '__main__':
    main()
