import serial
import struct
import time
import sys

# MSP V2 Message Structure
# Offset |    Usage     | CRC | Comment
#   0    |      $       | No  | Same lead-in as V1
#   1    |      X       | No  | 'X' in place of 'M' in V1
#   2    |     type     | No  | '<' = request, '>' = response, '!' = error 
#   3    |     flag     | Yes | uint8, usually 0
#   4    |   function   | Yes | uint16 (2 bytes, little-endian)
#   6    | payload size | Yes | uint16 (2 bytes, little-endian)
#   8    |   payload    | Yes | n bytes
#  n+8   |   checksum   | No  | uint8, crc8_dvb_s2 checksum

def calculate_DVB_S2_checksum(data) -> int:
    checksum = 0x00
    for byte in data:
        checksum ^= byte
        for _ in range(8):
            if checksum & 0x80: 
                checksum = (checksum << 1) ^ 0xD5
            else: 
                checksum <<= 1
            checksum &= 0xFF 
    return checksum

def create_msp_request(function):
    # Length is 9 bytes for a request with no payload
    message = [0]*9
    message[0] = ord('$')
    message[1] = ord('X')
    message[2] = ord('<')
    message[3] = 0                              # flag
    message[4] = function & 0xFF                # function low byte
    message[5] = (function >> 8) & 0xFF         # function high byte
    message[6] = 0                              # payload size low byte
    message[7] = 0                              # payload size high byte
    # Checksum is calculated on bytes 3 to 7 (inclusive)
    message[8] = calculate_DVB_S2_checksum(message[3:8])
    return bytes(message)

def read_msp_v2_response(ser, timeout=1.0):
    start_time = time.time()
    buffer = bytearray()
    
    # 1. Wait for header '$X>'
    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            buffer.extend(ser.read(ser.in_waiting))
            header_idx = buffer.find(b'$X>')
            if header_idx != -1:
                # Discard any bytes before the header
                buffer = buffer[header_idx:]
                break
        time.sleep(0.01)
    else:
        return None
        
    # 2. Wait for at least the 8-byte header to be fully read
    while len(buffer) < 8 and (time.time() - start_time < timeout):
        if ser.in_waiting > 0:
            buffer.extend(ser.read(ser.in_waiting))
        time.sleep(0.01)
        
    if len(buffer) < 8:
        return None
        
    # Parse header details
    flag = buffer[3]
    function = struct.unpack('<H', buffer[4:6])[0]
    payload_size = struct.unpack('<H', buffer[6:8])[0]
    
    # 3. Wait for the payload and the 1-byte checksum to arrive
    expected_len = 8 + payload_size + 1
    while len(buffer) < expected_len and (time.time() - start_time < timeout):
        if ser.in_waiting > 0:
            buffer.extend(ser.read(ser.in_waiting))
        time.sleep(0.01)
        
    if len(buffer) < expected_len:
        return None
        
    # Extract payload and checksum
    payload = buffer[8 : 8 + payload_size]
    checksum = buffer[8 + payload_size]
    
    # Verify CRC on the data block (header from index 3 up to end of payload)
    crc_data = buffer[3 : 8 + payload_size]
    calculated_crc = calculate_DVB_S2_checksum(crc_data)
    
    if calculated_crc != checksum:
        print(f"❌ CRC check failed. Got: {checksum}, Calculated: {calculated_crc}")
        return None
        
    return function, payload

def main():
    SERIAL_PORT = '/dev/serial0'
    BAUD_RATE = 115200
    
    MSP_RAW_IMU = 102
    MSP_RC = 105
    
    print("================================================================")
    print("      MSP V2 Telemetry Test Utility for Autonomous UAV Interceptor")
    print("================================================================")
    print(f"เชื่อมต่อพอร์ต {SERIAL_PORT} ที่ {BAUD_RATE} baud...")
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
    except Exception as e:
        print(f"❌ ไม่สามารถเปิดพอร์ตซีเรียลได้: {e}")
        return

    print("กรุณาตรวจสอบว่า:")
    print("  1. ในหน้า Ports ของ INAV Configurator ได้เปิดสวิตช์สีฟ้า MSP บน UART2 และกด Save เรียบร้อยแล้ว")
    print("  2. ถ้าโค้ดไม่ตอบรับ ให้ตรวจสอบว่าปิดการรันโปรแกรม MAVLink อื่นๆ แล้ว")
    
    print("\nเริ่มดึงข้อมูล IMU (กด Ctrl+C เพื่อหยุด)...")
    time.sleep(1)
    
    try:
        while True:
            # Create and send MSP V2 request for IMU
            msp_request = create_msp_request(MSP_RAW_IMU)
            ser.reset_input_buffer()  # Clear old serial buffers
            ser.write(msp_request)
            
            # Read and parse response
            res = read_msp_v2_response(ser)
            if res:
                function, payload = res
                if function == MSP_RAW_IMU:
                    ax, ay, az = struct.unpack('<hhh', payload[:6])
                    accel_scale = 512.0
                    ax_g, ay_g, az_g = ax / accel_scale, ay / accel_scale, az / accel_scale
                    
                    gx, gy, gz = struct.unpack('<hhh', payload[6:12])
                    gyro_scale = 4.0
                    gx_dps, gy_dps, gz_dps = gx / gyro_scale, gy / gyro_scale, gz / gyro_scale
                    
                    print(f"ACC: X={ax_g:+.3f}g, Y={ay_g:+.3f}g, Z={az_g:+.3f}g | GYRO: X={gx_dps:+.1f}dps, Y={gy_dps:+.1f}dps, Z={gz_dps:+.1f}dps")
            else:
                print("⚠️  ไม่ได้รับการตอบรับจาก Flight Controller (Timeout)")
                
            time.sleep(0.2)
            
    except KeyboardInterrupt:
        print("\nหยุดการทำงานโดยผู้ใช้")
        ser.close()

if __name__ == '__main__':
    main()
