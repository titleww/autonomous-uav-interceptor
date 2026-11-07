# Code to test MAVLink telemetry communication between Pi and FC (running INAV or ArduPilot)
import time
import sys
from pymavlink import mavutil

def main():
    port = '/dev/serial0'
    baud = 115200
    
    print("================================================================")
    print("      MAVLink Communication Test Utility for Autonomous UAV Interceptor")
    print("================================================================")
    print(f"กำลังเชื่อมต่อพอร์ต {port} ที่ {baud} baud เพื่อรอรับแพ็กเกจ MAVLink...")
    
    try:
        master = mavutil.mavlink_connection(port, baud=baud)
    except Exception as e:
        print(f"❌ ไม่สามารถเปิดพอร์ตซีเรียลได้: {e}")
        return
        
    print("\n[ขั้นตอนที่ 1] รอรับสัญญาณ Heartbeat (บ่งบอกว่าสายเชื่อมต่อถูกต้อง)...")
    print("กรุณาตรวจสอบว่า:")
    print("  1. ในหน้า Ports ของ INAV Configurator ได้ตั้ง UART2 เป็น Telemetry -> MAVLink และ Baudrate เป็น 115200 หรือยัง?")
    print("  2. ต่อสายไฟ RX/TX ไขว้กันเรียบร้อยและ FC มีไฟเลี้ยงอยู่หรือไม่?")
    
    # Wait for heartbeat with a 15-second timeout
    heartbeat = master.wait_heartbeat(timeout=15.0)
    if heartbeat is not None:
        print(f"✅ เชื่อมต่อสำเร็จ! ได้รับ Heartbeat จาก System ID: {master.target_system}, Component ID: {master.target_component}")
        print(f"   เวอร์ชัน MAVLink ที่ใช้งาน: {master.mavlink10()}")
    else:
        print("❌ ไม่ได้รับ Heartbeat ภายใน 15 วินาที")
        print("   ไม่พบสัญญาณส่งออกจาก Flight Controller โปรดตรวจสอบการตั้งค่าพอร์ตและสายสัญญาณ")
        return
        
    print("\n[ขั้นตอนที่ 2] จำลองการอ่านค่า Telemetry จริงจาก Flight Controller เป็นเวลา 10 วินาที...")
    start_time = time.time()
    msg_counts = {}
    
    try:
        while time.time() - start_time < 10.0:
            msg = master.recv_match(blocking=False)
            if msg is not None:
                msg_type = msg.get_type()
                msg_counts[msg_type] = msg_counts.get(msg_type, 0) + 1
                
                # แสดงตัวอย่างข้อมูลสำคัญบางประเภท เช่น ATTITUDE, VFR_HUD, SYS_STATUS
                if msg_type == 'ATTITUDE':
                    roll = msg.roll * 57.2958  # rad to deg
                    pitch = msg.pitch * 57.2958
                    yaw = msg.yaw * 57.2958
                    print(f"-> Telemetry: Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
                elif msg_type == 'SYS_STATUS':
                    voltage = msg.voltage_battery / 1000.0  # mV to V
                    current = msg.current_battery / 100.0   # 10mA to A
                    print(f"-> Power: Battery Voltage: {voltage:.2f}V, Current: {current:.2f}A")
                elif msg_type == 'VFR_HUD':
                    print(f"-> Flight Info: Altitude: {msg.alt:.2f}m, Heading: {msg.heading}°")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nหยุดการทดสอบโดยผู้ใช้")
        
    print("\n================================================================")
    print("สรุปแพ็กเกจ MAVLink ที่ได้รับ:")
    if msg_counts:
        for m_type, count in msg_counts.items():
            print(f" - {m_type}: {count} แพ็กเกจ")
        print("\n🎉 การทดสอบเสร็จสมบูรณ์! การสื่อสาร MAVLink (Telemetry) ทำงานได้ปกติ")
    else:
        print(" - ไม่พบแพ็กเกจข้อมูลอื่นๆ นอกจาก Heartbeat")
        print("⚠️  ตรวจสอบให้แน่ใจว่าได้บันทึกการตั้งค่าพอร์ตใน FC แล้ว")
    print("================================================================")

if __name__ == '__main__':
    main()
