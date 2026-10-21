from adafruit_servokit import ServoKit
import time
from gpiozero import PWMOutputDevice, DigitalInputDevice

# BLDC motor
BLDC_speed_pin = PWMOutputDevice(18) # Blue wire
BLDC_encoder_pin = DigitalInputDevice(27, pull_up=True) # Yellow FG
# Servo motor
servo_pin=0
# Initialize the ServoKit with 16 channels and the default I2C address (0x40)
kit = ServoKit(channels=16)
kit.frquency = 100

def BLDC_set_speed(value):
    BLDC_speed_pin.value = value

while True:
    # BLDC 
    BLDC_set_speed(0) 
    time.sleep(2)
    BLDC_set_speed(1)
    time.sleep(2)

    # Servo
    # Move to 0 degrees
    kit.servo[servo_pin].angle = 0
    time.sleep(1)
    # Move to 90 degrees
    kit.servo[servo_pin].angle = 30
    time.sleep(1)
    # Move to 180 degrees
    kit.servo[servo_pin].angle = 0
    time.sleep(1)
