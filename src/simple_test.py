# โค้ดทดสอบหมุนมอเตอร์ (Autonomous UAV Interceptor)
import time
from pymavlink import mavutil



print("1. กำลังเชื่อมต่อกับ Flight Controller...")
master = mavutil.mavlink_connection('/dev/serial0', baud=115200)

print("2. รอการตอบรับจากบอร์ด (Heartbeat)...")
master.wait_heartbeat()
print("เชื่อมต่อสำเร็จ!")

# สั่งหมุนมอเตอร์ที่ 1 ด้วยความเร็ว 10% เป็นเวลา 3 วินาที
print("3. กำลังส่งคำสั่งทดสอบมอเตอร์ที่ 1 (10% เป็นเวลา 3 วินาที)...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, # คำสั่งทดสอบมอเตอร์
    0,      # confirmation
    1,      # หมายเลขมอเตอร์ (1-4)
    0,      # ประเภทการระบุแรงบิด (0 = เปอร์เซ็นต์)
    10,     # แรงบิดความเร็ว (10%)
    3,      # เวลาในการหมุน (3 วินาที)
    0, 0, 0, 0 # พารามิเตอร์เสริม
)
print(master.mav.command_long_send)
print("4. รอข้อมูลตอบกลับ (COMMAND_ACK) จากบอร์ด...")
ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
if ack:
    print(f"ผลตอบกลับจาก FC (ACK Message): {ack}")
    print(f"-> Command ID: {ack.command} (320 = DO_MOTOR_TEST)")
    print(f"-> Result Code: {ack.result} (0 = ACCEPTED / สำเร็จ, 4 = FAILED / ล้มเหลว)")
else:
    print("-> ไม่ได้รับข้อความตอบกลับภายใน 3 วินาที")

print("ส่งคำสั่งเรียบร้อย! สังเกตว่ามอเตอร์ตัวที่ 1 หมุนหรือไม่")
