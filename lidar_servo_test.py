import serial
import time
from adafruit_servokit import ServoKit
from gpiozero import PWMOutputDevice, DigitalInputDevice

# ==========================================
# 1. Hardware Initialization
# ==========================================

# -- Lidar Setup (Benewake TF-Luna) --
ser = serial.Serial("/dev/serial0", 115200, timeout=0)

# -- BLDC Motor Setup --
BLDC_speed_pin = PWMOutputDevice(18)  # Blue wire
BLDC_encoder_pin = DigitalInputDevice(27, pull_up=True)  # Yellow FG

# -- Servo Motor Setup --
servo_pin = 0
kit = ServoKit(channels=16)
kit.frequency = 50
# ==========================================
# 2. Helper Functions
# ==========================================

def read_tfluna_data():
    """Reads and parses the 9-byte data frame from the TF-Luna Lidar."""
    while True:
        counter = ser.in_waiting
        if counter > 8:
            bytes_serial = ser.read(9)
            ser.reset_input_buffer()
            # 
            # FIX THIS SERIAL ADDRESS PART, MAKE SURE IT IS IN THE CORRECT SPOT
            # TEMPRATURE IS FOR THE TEMPRATURE OF THE LIDAR MODULE ITSELF
            # SIGNAL STRENGTH IS INTENSITY OF THE RETURNING LIGHT PULSE SENT BY THE LIDAR
            # DISTANCE IS JUST DISTANCE FROM GROUND (NEED TO MEASURE HEIGHT OF LIDAR MOUNT)
            #
            # 0x59 0x59 is the standard Benewake frame header
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                distance = bytes_serial[2] + bytes_serial[3] * 256
                strength = bytes_serial[4] + bytes_serial[5] * 256
                temperature = bytes_serial[6] + bytes_serial[7] * 256
                temperature = (temperature / 8.0) - 256.0

                # Returns distance in meters, signal strength, and temperature
                return distance / 100.0, strength, temperature

def BLDC_set_speed(value):
    """Sets the PWM value for the BLDC motor (0.0 to 1.0)."""
    BLDC_speed_pin.value = value

# ==========================================
# 3. Main Logic Loop
# ==========================================

# Open serial port if not already open
if not ser.isOpen():
    ser.open()

# Constants for logic
GROUND_THRESHOLD = 0.25  # Set to 25 cm (0.25 meters) based on mount position
CRUISE_SPEED = 0.5       # Motor speed while in the air (0.0 to 1.0)

print("Starting system... Press Ctrl+C to stop.")

try:
    while True:
        # 1. Read the distance
        distance, strength, temperature = read_tfluna_data()
        
        print("Distance: {0:2.2f} m | Threshold: {1:2.2f} m".format(distance, GROUND_THRESHOLD))

        # 2. Evaluate distance and control the motor
        # If the reading is 25cm (0.25m) or less, we've hit the ground.
        if distance <= GROUND_THRESHOLD:
            print("Ground distance reached! Shutting off motor.")
            BLDC_set_speed(0)
            
            # Optional: Set servo to a specific "landed" angle
            kit.servo[servo_pin].angle = 0 
            
            # Break out of the loop since we've hit the ground
            break 
            
        else:
            # We are still in the air (distance > 0.25m)
            BLDC_set_speed(CRUISE_SPEED)
            
            # Optional: Keep servo at a "flying" angle
            kit.servo[servo_pin].angle = 90

        # Small delay for system stability and responsiveness
        time.sleep(0.1) 

except KeyboardInterrupt:
    print("\nManual stop triggered...")

finally:
    # 4. Graceful Shutdown & Safety
    print("Cleaning up hardware states.")
    BLDC_set_speed(0) # Ensure the motor is turned off when the script exits
    ser.close()       # Close the serial connection