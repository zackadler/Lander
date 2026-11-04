import time
from smbus import SMBus

# ================= SETTINGS YOU CAN CHANGE =================

MOTOR_1 = 6           # HAT channel number (0-15) for motor 1 (CW)
MOTOR_2 = 7           # HAT channel number (0-15) for motor 2 (CCW)

STOP_US = 1000        # microseconds (us): pulse length that means "stop"  (0% throttle)
FULL_US = 2500        # microseconds (us): pulse length that means "full"  (100% throttle, measured on your setup)
SPEED_US = 1850      # microseconds (us): pulse length the motors run at
                      #   throttle % is roughly (SPEED_US - 1000) / 15, so 1600 us = about 40%

ARM_TIME_S = 5        # seconds: how long to send "stop" so the ESCs arm before spinning

# ================= HAT SETTINGS (don't change) =================

bus = SMBus(1)        # I2C bus number on the Pi (bus 1 = pins 3 and 5, where the HAT talks)
HAT = 0x40            # I2C address of the HAT (the "40" in i2cdetect)

PERIOD_US = 20000     # microseconds (us): time between the start of one pulse and the next
                      #   20,000 us = 20 ms = 50 pulses per second
STEPS = 4096          # steps: the HAT splits each 20,000 us period into 4096 equal steps
                      #   so 1 step = 20,000 / 4096 = about 4.9 us
PRESCALE = 121        # no unit: divider that slows the HAT's 25,000,000 Hz clock down to 50 Hz
                      #   25,000,000 / (4096 x 50) - 1 = about 121

# HAT register numbers (memory slots inside the HAT's chip)
MODE1 = 0x00          # main settings slot (sleep / awake)
PRE_SCALE = 0xFE      # pulse-rate slot (holds PRESCALE)
CH0_START = 0x06      # first slot for channel 0; each channel uses 4 slots after this
SLEEP = 0x10          # value for MODE1 that puts the chip to sleep
AWAKE = 0x00          # value for MODE1 that wakes the chip up
FULL_OFF = 0x10       # value that turns a channel's output completely off


def set_pulse(channel, microseconds):
    """Make `channel` send pulses that stay on for `microseconds` (us)."""
    ticks = round(microseconds * STEPS / PERIOD_US)   # steps: pulse length converted to HAT steps
    reg = CH0_START + 4 * channel                     # register number of this channel's first slot
    bus.write_byte_data(HAT, reg + 0, 0)              # ON step, low part:  pulse turns on at step 0
    bus.write_byte_data(HAT, reg + 1, 0)              # ON step, high part
    bus.write_byte_data(HAT, reg + 2, ticks & 0xFF)   # OFF step, low part  (remainder after 256s)
    bus.write_byte_data(HAT, reg + 3, ticks >> 8)     # OFF step, high part (how many 256s)


def signal_off(channel):
    """Stop sending pulses on `channel` (the wire stays at 0 V)."""
    bus.write_byte_data(HAT, CH0_START + 4 * channel + 3, FULL_OFF)


# ---- set the HAT to 50 pulses per second ----
bus.write_byte_data(HAT, MODE1, SLEEP)        # chip must sleep to change the pulse rate
bus.write_byte_data(HAT, PRE_SCALE, PRESCALE) # set the rate to 50 pulses per second
bus.write_byte_data(HAT, MODE1, AWAKE)        # wake it up
time.sleep(0.01)                              # seconds: give the chip's clock time to start

# ---- run both motors ----
try:
    set_pulse(MOTOR_1, STOP_US)               # send "stop" so both ESCs arm
    set_pulse(MOTOR_2, STOP_US)
    print("Arming both ESCs for", ARM_TIME_S, "seconds...")
    time.sleep(ARM_TIME_S)

    set_pulse(MOTOR_1, SPEED_US)              # spin both motors
    set_pulse(MOTOR_2, SPEED_US)
    percent = round((SPEED_US - STOP_US) / (FULL_US - STOP_US) * 100)
    input("Both motors at " + str(SPEED_US) + " us (about " + str(percent) + "% throttle). Press Enter to stop.")
finally:
    set_pulse(MOTOR_1, STOP_US)               # always stop the motors, even after Ctrl+C
    set_pulse(MOTOR_2, STOP_US)
    time.sleep(1)                             # seconds: let the ESCs see "stop"
    signal_off(MOTOR_1)
    signal_off(MOTOR_2)
    print("Both motors stopped.")