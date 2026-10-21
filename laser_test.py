import serial
import time

# Initialize serial with a 1-second timeout. 
try:
    ser = serial.Serial("/dev/serial0", 115200, timeout=1)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

def read_tfluna_data():
    # CRITICAL FIX: Flush the old data out of the buffer first.
    # This ensures we are catching the freshest 100Hz frame after our 0.5s sleep.
    ser.reset_input_buffer()
    
    while True:
        # 1. Look for the first header byte (0x59)
        if ser.read(1) == b'\x59':
            
            # 2. Look for the second header byte (0x59)
            if ser.read(1) == b'\x59':
                
                # 3. Headers found! Read the remaining 7 bytes of the frame
                payload = ser.read(7)
                
                if len(payload) == 7:
                    # 4. Construct the full 9-byte frame to check data integrity
                    frame = [0x59, 0x59] + list(payload)
                    
                    # Checksum is the sum of the first 8 bytes, keeping only the lowest 8 bits
                    checksum = sum(frame[:8]) & 0xFF
                    
                    if frame[8] == checksum:
                        # 5. Data is valid, calculate outputs
                        distance = frame[2] + (frame[3] << 8)
                        strength = frame[4] + (frame[5] << 8)
                        temperature = frame[6] + (frame[7] << 8)
                        
                        # Apply TF-Luna temperature formula
                        temperature = (temperature / 8.0) - 256.0
                        
                        # Return distance in meters, strength, and temp
                        return distance / 100.0, strength, temperature
                    else:
                        # Checksum failed, loop again to find the next valid frame
                        continue

try:
    print("Starting TF-Luna LiDAR...")
    while True:
        data = read_tfluna_data()
        
        if data:
            distance, strength, temperature = data
            print(
                f"Distance: {distance:2.2f} m, "
                f"Strength: {strength:5d} / 65535, "
                f"Chip Temperature: {temperature:2.1f} C"
            )
        
        # Note: The TF-Luna outputs at 100Hz by default. 
        # Sleeping for 0.5s is now safe because we flush the buffer on the next read.
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    if ser.is_open:
        ser.close()