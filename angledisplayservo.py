import random
import smbus                    # import SMBus module of I2C
import math
from time import sleep
from adafruit_servokit import ServoKit
import time

# Servo motor
servo_roll = 5
servo_pitch = 4

# Initialize the ServoKit with 16 channels, the default I2C address (0x40), and 100 Hz PWM
kit = ServoKit(channels=16, frequency=100)

# range of motion is 0 to 30 degrees for channel 4/5 or 1/2 with the other PCB
CENTER = 15   # servo angle where the gimbal is at neutral
LIMIT = 15    # max deflection either side of CENTER


def servo_name(channel):
    if channel == servo_roll:
        return "roll"
    if channel == servo_pitch:
        return "pitch"
    return f"ch{channel}"


def deflect(channel, offset):
    offset = max(-LIMIT, min(LIMIT, offset))  # never leave the safe range
    kit.servo[channel].angle = CENTER + offset
    actual = kit.servo[channel].angle  # commanded angle read back from the PWM driver
    print(f"[{time.strftime('%H:%M:%S')}] {servo_name(channel)} (ch {channel}): "
          f"offset {offset:+.1f}°, angle {actual:.1f}°", flush=True)


test_servo = 4  # change to servo_pitch (4) or servo_roll (5) to test for certain ranges.

# Center both servos at startup so every channel has a known angle
deflect(servo_roll, 0)
deflect(servo_pitch, 0)
sleep(1)

try:
    while True:
        deflect(test_servo, LIMIT)
        sleep(1)

        deflect(test_servo, 0)
        sleep(1)

        deflect(test_servo, -LIMIT)
        sleep(1)

        deflect(test_servo, 0)
        sleep(1)
except KeyboardInterrupt:
    # Return to neutral on Ctrl+C
    print("Stopping, returning to center...", flush=True)
    deflect(servo_roll, 0)
    deflect(servo_pitch, 0)