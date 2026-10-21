import random
import smbus                    #import SMBus module of I2C
import math
from time import sleep
from adafruit_servokit import ServoKit
import time

# Servo motor
servo_roll=5
servo_pitch=4

# Initialize the ServoKit with 16 channels, the default I2C address (0x40), and 100 Hz PWM
kit = ServoKit(channels=16, frequency=100)

# range of motion is 0 to 30 degrees for channel 4/5 or 1/2 with the other PCB
CENTER = 15   # servo angle where the gimbal is at neutral
LIMIT = 15    # max deflection either side of CENTER

def deflect(channel, offset):
    offset = max(-LIMIT, min(LIMIT, offset))  # never leave the safe range
    kit.servo[channel].angle = CENTER + offset

test_servo = 4  # change to servo_pitch(4) or servo_roll(5) to test for certain ranges.

while True:
    deflect(test_servo, LIMIT)
    sleep(1)

    deflect(test_servo, 0)
    sleep(1)

    deflect(test_servo, -LIMIT)
    sleep(1)