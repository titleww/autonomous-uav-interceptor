# โค้ดทดสอบการเชื่อมต่อและหมุนมอเตอร์ผ่านโปรโตคอล MSP สำหรับ INAV
import time
import struct
import serial
import sys

# ⚠️ คำเตือนความปลอดภัย: ถอดใบพัดออกทั้งหมดก่อนใช้งาน!

# รหัสคำสั่ง MSP (MSP Command IDs)
MSP_API_VERSION   = 1
MSP_IDENT         = 100
MSP_SET_RAW_RC    = 200

def robust_input(prompt):
    """ฟังก์ชันรับค่าอินพุตคีย์บอร์ดที่ป้องกันปัญหา UnicodeDecodeError"""
    sys.stdout.write(prompt)
    sys.stdout.flush()
    try:
        # อ่านค่าเป็น Raw bytes และทำการถอดรหัสแบบข้ามตัวอักษรที่ผิดพลาด (ignore error)
        line = sys.stdin.buffer.readline()
        return line.decode('utf-8', errors='ignore').strip()
    except Exception:
        # Fallback หากระบบแชร์สตรีมไม่ได้
        try:
            return input().strip()
        except Exception:
            return ""

def send_msp(ser, cmd, payload=b""):
    """สร้างและส่งแพ็คเกจ MSP V1"""
    size = len(payload)
    checksum = size ^ cmd
    for b in payload:
        checksum ^= b
        
    # โครงสร้าง MSP V1: '$', 'M', '<', size, cmd, payload..., checksum
    packet = bytearray([0x24, 0x4D, 0x3C, size, cmd])
    packet.extend(payload)
    packet.append(checksum)
    
    ser.write(packet)
    ser.flush()

def read_msp_response(ser, expected_cmd, timeout=1.0):
    """รอรับข้อมูลตอบกลับ MSP V1 จากบอร์ดควบคุม"""
    start_time = time.time()
    buffer = bytearray()
    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            buffer.extend(ser.read(ser.in_waiting))
            
            # ค้นหาส่วนหัวของแพ็คเกจตอบกลับ '$M>'
            header_idx = buffer.find(b'$M>')
            if header_idx != -1 and len(buffer) >= header_idx + 6:
                size = buffer[header_idx + 3]
                cmd = buffer[header_idx + 4]
                if len(buffer) >= header_idx + 6 + size:
                    payload = buffer[header_idx + 5 : header_idx + 5 + size]
                    checksum = buffer[header_idx + 5 + size]
                    
                    # ตรวจสอบ Checksum
                    calc_checksum = size ^ cmd
                    for b in payload:
                        calc_checksum ^= b
                        
                    if calc_checksum == checksum:
                        return cmd, payload
                    # ล้างข้อมูลที่ประมวลผลแล้ว
                    buffer = buffer[header_idx + 6 + size:]
        time.sleep(0.01)
    return None, None

def main():
    print("================================================================")
    print("         MSP Connection & Test Utility for INAV (Autonomous UAV Interceptor)")
    print("================================================================")
    print("⚠️  CRITICAL SAFETY WARNING: REMOVE ALL PROPELLERS BEFORE RUNNING!")
    print("================================================================")
    
    port = '/dev/serial0'
    baud = 115200
    
    print(f"กำลังเชื่อมต่อกับ FC ผ่านซีเรียลพอร์ต {port} ({baud} baud)...")
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except Exception as e:
        print(f"เชื่อมต่อพอร์ตซีเรียลล้มเหลว: {e}")
        return

    # 1. ทดสอบส่งข้อความเช็คสัญญาน (Ping ด้วยคำสั่ง MSP_API_VERSION)
    print("1. กำลังส่งสัญญาณตรวจสอบการเชื่อมต่อ (Ping) ไปยัง INAV...")
    send_msp(ser, MSP_API_VERSION)
    cmd, payload = read_msp_response(ser, MSP_API_VERSION)
    
    if cmd == MSP_API_VERSION and payload:
        msp_protocol_ver = payload[0]
        api_major = payload[1]
        api_minor = payload[2]
        print(f"-> การเชื่อมต่อสำเร็จ! พบข้อมูลตอบกลับจาก INAV")
        print(f"   [MSP Protocol Ver: {msp_protocol_ver}, API Version: {api_major}.{api_minor}]")
    else:
        print("-> ไม่ได้รับการตอบกลับจากบอร์ด (No response received)")
        print("   กรุณาตรวจสอบว่า:")
        print("   1. ต่อสาย RX ของ Pi เข้ากับ TX ของ FC และ TX ของ Pi เข้ากับ RX ของ FC หรือยัง?")
        print("   2. บอร์ด FC เปิดใช้งานสวิตช์ MSP บน UART2 และตั้งค่าเป็น 115200 หรือยัง?")
        print("   3. บอร์ด FC มีไฟเลี้ยงอยู่หรือไม่?")
        return

    # 2. เข้าสู่เมนูตัวเลือกการทดสอบ
    while True:
        print("\nตัวเลือกการทดสอบ:")
        print("1. ส่งพิกัดวิทยุจำลอง (Test Virtual RC Channels) - ปลอดภัยสูง เพื่อเช็คแถบสัญญาณในแถบ Receiver")
        print("2. ทดสอบสั่ง Arm และหมุนมอเตอร์เดี่ยว (ความปลอดภัยสูงสุด - ต้องถอดใบพัด!)")
        print("3. ออกจากโปรแกรม")
        
        choice = robust_input("กรุณาเลือกตัวเลือก (1-3): ")
        
        if choice == '1':
            print("\n*กำลังส่งสัญญาณวิทยุจำลองเป็นเวลา 10 วินาที...*")
            print("เปิดโปรแกรม INAV Configurator ไปที่แท็บ 'Receiver' คุณจะเห็นแถบสัญญาณช่องต่างๆ ขยับตามคลื่น")
            
            start_run = time.time()
            step = 0
            try:
                while time.time() - start_run < 10.0:
                    # สร้างคลื่นจำลองให้ช่อง Roll (1) และ Pitch (2) ขยับขวา-ซ้าย
                    # จำลองช่อง: Roll, Pitch, Yaw, Throttle, AUX1, AUX2, AUX3, AUX4
                    # ค่าช่วง 1000 - 2000, ค่ากลางคือ 1500, Throttle อยู่ที่ 1000 (ต่ำสุด)
                    roll = int(1500 + 100 * (step % 20 - 10) / 10) # ส่ายเลี้ยวซ้ายขวา
                    channels = [roll, 1500, 1500, 1000, 1000, 1000, 1000, 1000]
                    
                    payload = struct.pack('<8H', *channels)
                    send_msp(ser, MSP_SET_RAW_RC, payload)
                    
                    step += 1
                    time.sleep(0.02) # ส่งด้วยความถี่ 50Hz
                print("เสร็จสิ้นการส่งสัญญาณวิทยุจำลอง")
            except KeyboardInterrupt:
                print("\nหยุดการส่งสัญญาณจำลอง")
                
        elif choice == '2':
            confirm = robust_input("คุณแน่ใจแล้วใช่ไหมว่าได้ 'ถอดใบพัดโดรนออกทั้งหมดแล้ว'? (yes/no): ").lower()
            if confirm == 'yes':
                print("\n*เริ่มขั้นตอนจำลองคำสั่งวิทยุเพื่อสั่ง Arm และเร่งมอเตอร์เบาๆ...*")
                print("หมายเหตุ: เพื่อให้คำสั่งนี้ทำงานได้ คุณต้องตั้งค่าโหมด ARM ให้ทำงานกับ AUX1 (Channel 5) ในแท็บ Modes ของ INAV ล่วงหน้า")
                
                # ลำดับการทดสอบที่ถูกต้องและปลอดภัยสำหรับบอร์ด FC:
                # 1. ส่งสัญญาณ Disarm ก่อน 1 วินาที (AUX1 = 1000, Throttle = 1000) เพื่อเริ่มระบบใหม่
                print("-> 1/4 ส่งสัญญาณเตรียมความพร้อม (Disarmed)...")
                for _ in range(50):
                    channels = [1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000]
                    send_msp(ser, MSP_SET_RAW_RC, struct.pack('<8H', *channels))
                    time.sleep(0.02)
                
                # 2. ปลดล็อกบอร์ด (ARM: AUX1 = 2000) แต่ห้ามเร่งคันเร่ง (Throttle = 1000 ต่ำสุด) นาน 0.5 วินาที
                # เนื่องจาก FC ส่วนใหญ่จะไม่ยอม ARM หากคันเร่งไม่อยู่ที่จุดต่ำสุด (Safety Check)
                print("-> 2/4 ส่งสัญญาณ ARM ปลดล็อกความปลอดภัย (Throttle Low)...")
                for _ in range(25):
                    channels = [1500, 1500, 1500, 1000, 2000, 1000, 1000, 1000]
                    send_msp(ser, MSP_SET_RAW_RC, struct.pack('<8H', *channels))
                    time.sleep(0.02)
                
                # 3. เมื่อปลดล็อกแล้ว จึงค่อยเร่งคันเร่งขึ้นเล็กน้อย (Throttle = 1150) เพื่อให้มอเตอร์หมุนค้างไว้ 3 วินาที
                print("-> 3/4 เร่งความเร็วมอเตอร์เป็นเวลา 3 วินาที (Spinning)...")
                for _ in range(150):
                    channels = [1500, 1500, 1500, 1150, 2000, 1000, 1000, 1000]
                    send_msp(ser, MSP_SET_RAW_RC, struct.pack('<8H', *channels))
                    time.sleep(0.02)
                
                # 4. สั่งล็อกความปลอดภัยทันทีก่อนจบโปรแกรม (AUX1 = 1000, Throttle = 1000)
                print("-> 4/4 สั่งล็อกมอเตอร์ความปลอดภัยทันที (Disarmed)...")
                for _ in range(50):
                    channels = [1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000]
                    send_msp(ser, MSP_SET_RAW_RC, struct.pack('<8H', *channels))
                    time.sleep(0.02)
                
                print("เสร็จสิ้นการทดสอบมอเตอร์")
            else:
                print("ยกเลิกการทดสอบเพื่อความปลอดภัย")
                
        elif choice == '3':
            # ตรวจสอบให้แน่ใจว่า Disarm ก่อนออก
            for _ in range(10):
                channels = [1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000]
                send_msp(ser, MSP_SET_RAW_RC, struct.pack('<8H', *channels))
                time.sleep(0.01)
            print("ปิดการใช้งานพอร์ตซีเรียล...")
            ser.close()
            break
        else:
            print("ตัวเลือกไม่ถูกต้อง")

if __name__ == '__main__':
    main()
