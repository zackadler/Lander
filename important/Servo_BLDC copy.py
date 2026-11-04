from adafruit_servokit import ServoKit
import time

# Servo motor
servo_pin = 4

# Initialize the ServoKit with 16 channels and the default I2C address (0x40)
kit = ServoKit(channels=16)
kit.frquency = 100

while True:
    # Servo
    # Move to 0 degrees
    kit.servo[servo_pin].angle = 0
    time.sleep(1)
    # Move to 90 degrees
    kit.servo[servo_pin].angle = 30
    time.sleep(1)
 
