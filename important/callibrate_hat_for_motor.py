import time
from smbus import SMBus

bus = SMBus(1)
HAT = 0x40
CHANNEL = 6

# same setup as spin.py: 50 pulses per second
bus.write_byte_data(HAT, 0x00, 0x10)
bus.write_byte_data(HAT, 0xFE, 121)
bus.write_byte_data(HAT, 0x00, 0x00)
time.sleep(0.01)

def set_pulse(microseconds):
    ticks = round(microseconds * 4096 / 20000)
    reg = 0x06 + 4 * CHANNEL
    bus.write_byte_data(HAT, reg + 0, 0)
    bus.write_byte_data(HAT, reg + 1, 0)
    bus.write_byte_data(HAT, reg + 2, ticks & 0xFF)
    bus.write_byte_data(HAT, reg + 3, ticks >> 8)

def signal_off():
    bus.write_byte_data(HAT, 0x06 + 4 * CHANNEL + 3, 0x10)

try:
    input("PROPELLER OFF. Battery UNPLUGGED. Press Enter when ready...")

    set_pulse(2000)       # full speed: "this is the top"
    print("Sending FULL SPEED pulse. Plug in the battery now.")
    input("After the startup tones you'll hear 2 short beeps. Press Enter right after them...")

    set_pulse(1000)       # stop: "this is the bottom"
    print("Sending STOP pulse. The ESC should beep a few times, then one long beep.")
    time.sleep(5)
    print("Calibration saved.")

finally:
    set_pulse(1000)
    time.sleep(1)
    signal_off()
    print("Signal off. Unplug the battery.")