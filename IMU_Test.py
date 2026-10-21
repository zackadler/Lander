"""
Reads x/y/z acceleration, roll, and pitch from an ICM20948 IMU over
I2C bus 3 on a Raspberry Pi 5. The WHO_AM_I check confirmed this is
the actual chip on the board, not the MPU6050/9250 family the earlier
version assumed.

The ICM20948 splits its registers across four numbered banks, and
several addresses get reused for a different purpose in each one.
REG_BANK_SEL picks which bank is active before any other register
address means anything, which is why the previous version, written
for the MPU6050's single flat register map, was reading meaningless
values instead of real accelerometer data.

Axis convention, based on how the IMU is mounted on the platform:
    x = side to side, positive to the right
    y = forward / backward, positive moving forward
    z = up / down, positive moving up

Roll and pitch come from which way gravity leans in the accelerometer
reading, so they only stay accurate while the board is roughly still
or moving smoothly, a sudden shove will throw them off briefly since
the sensor can't yet tell that apart from an actual tilt. Roll reads
positive when the right side dips down, pitch reads positive when the
front lifts up.
"""

import math
import smbus
import time

BUS_NUM = 3
DEVICE_ADDRESS = 0x68

REG_BANK_SEL = 0x7F

# Bank 0
WHO_AM_I     = 0x00
PWR_MGMT_1   = 0x06
ACCEL_XOUT_H = 0x2D
ACCEL_YOUT_H = 0x2F
ACCEL_ZOUT_H = 0x31

# Bank 2
ACCEL_CONFIG = 0x14

G = 9.80665             # 1 g in m/s^2
ACCEL_SCALE = 16384.0   # LSB per g at the +/-2g range

bus = smbus.SMBus(BUS_NUM)


def select_bank(bank):
    bus.write_byte_data(DEVICE_ADDRESS, REG_BANK_SEL, (bank & 0x03) << 4)


def icm_init():
    select_bank(0)
    bus.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 0x01)    # wake up, auto clock select
    time.sleep(0.1)

    select_bank(2)
    bus.write_byte_data(DEVICE_ADDRESS, ACCEL_CONFIG, 0x00)  # +/-2g range, max sensitivity

    select_bank(0)   # leave it on bank 0, where the accelerometer data lives
    time.sleep(0.1)


def read_raw(addr):
    high = bus.read_byte_data(DEVICE_ADDRESS, addr)
    low = bus.read_byte_data(DEVICE_ADDRESS, addr + 1)
    value = (high << 8) | low
    if value >= 32768:
        value -= 65536
    return value


def read_accel_g():
    """Raw accelerometer reading in g's, one float per axis."""
    ax = read_raw(ACCEL_XOUT_H) / ACCEL_SCALE
    ay = read_raw(ACCEL_YOUT_H) / ACCEL_SCALE
    az = read_raw(ACCEL_ZOUT_H) / ACCEL_SCALE
    return ax, ay, az


def compute_roll_pitch(ax, ay, az):
    """
    Roll is the left/right tilt about the forward axis, positive when
    the right side dips down. Pitch is the forward/backward tilt about
    the side to side axis, positive when the front lifts up. Both come
    from where gravity points in this reading, so az here should still
    carry its resting 1g rather than have it subtracted out.
    """
    roll = math.degrees(math.atan2(-ax, az))
    pitch = math.degrees(math.atan2(ay, az))
    return roll, pitch


def calibrate(samples=1000):
    """
    Averages the accelerometer readings while the IMU sits flat and still.
    x and y should read 0g at rest, so their averages are pure bias.
    z reads +1g at rest from gravity, so that whole reading, gravity
    included, is stored as z's bias and subtracted out later against
    each live reading, the same way x and y are already handled.
    """
    print("Calibrating, keep the IMU flat and still...")
    sum_x = sum_y = sum_z = 0.0
    for _ in range(samples):
        ax, ay, az = read_accel_g()
        sum_x += ax
        sum_y += ay
        sum_z += az
        time.sleep(0.001)

    bias_x = sum_x / samples
    bias_y = sum_y / samples
    bias_z = sum_z / samples

    print(f"Calibration done. Bias  x={bias_x:.4f}g  y={bias_y:.4f}g  z={bias_z:.4f}g\n")
    return bias_x, bias_y, bias_z


def main():
    icm_init()
    bias_x, bias_y, bias_z = calibrate()

    print("Reading acceleration, roll, and pitch. Ctrl+C to stop.\n")

    while True:
        ax_g, ay_g, az_g = read_accel_g()

        # remove the calibrated bias, convert g's to m/s^2
        ax = (ax_g - bias_x) * G
        ay = (ay_g - bias_y) * G
        az = (az_g - bias_z) * G

        # roll/pitch need gravity still present, so only x and y get
        # their small sensor bias removed here, z keeps its full reading
        roll, pitch = compute_roll_pitch(ax_g - bias_x, ay_g - bias_y, az_g)

        print(
            f"{ax:6.2f} m/s^2 x  {ay:6.2f} m/s^2 y  {az:6.2f} m/s^2 z   |   "
            f"{roll:6.1f} deg roll  {pitch:6.1f} deg pitch"
        )

        time.sleep(0.5)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")