"""
ICM20948 IMU on I2C bus 3, refactored from IMU_Test.py into a class
you construct once (which also runs the one-time calibration) and
then call .read() on whenever you want a fresh reading -- no internal
loop or sleep, so the caller controls the timing.
"""

import math
import time

import smbus


class ImuSensor:
    """Bias-corrected acceleration (m/s^2) plus roll/pitch (degrees)
    from an ICM20948.

    Axis convention:
        x = side to side, positive to the right
        y = forward / backward, positive moving forward
        z = up / down, positive moving up

    Roll and pitch come from which way gravity leans in the
    accelerometer reading, so they only stay accurate while the board
    is roughly still or moving smoothly -- a sudden shove throws them
    off briefly, since the sensor can't yet tell that apart from an
    actual tilt. Roll reads positive when the right side dips down,
    pitch reads positive when the front lifts up.
    """

    REG_BANK_SEL = 0x7F

    # Bank 0
    WHO_AM_I = 0x00
    PWR_MGMT_1 = 0x06
    ACCEL_XOUT_H = 0x2D
    ACCEL_YOUT_H = 0x2F
    ACCEL_ZOUT_H = 0x31

    # Bank 2
    ACCEL_CONFIG = 0x14

    G = 9.80665             # 1 g in m/s^2
    ACCEL_SCALE = 16384.0   # LSB per g at the +/-2g range

    def __init__(self, bus_num=3, address=0x68, calibration_samples=1000):
        self.address = address
        self.bus = smbus.SMBus(bus_num)
        self._init_chip()
        self.bias_x, self.bias_y, self.bias_z = self._calibrate(calibration_samples)

    def _select_bank(self, bank):
        self.bus.write_byte_data(self.address, self.REG_BANK_SEL, (bank & 0x03) << 4)

    def _init_chip(self):
        self._select_bank(0)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x01)  # wake up, auto clock select
        time.sleep(0.1)

        self._select_bank(2)
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)  # +/-2g range, max sensitivity

        self._select_bank(0)  # leave it on bank 0, where the accelerometer data lives
        time.sleep(0.1)

    def _read_raw(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        value = (high << 8) | low
        if value >= 32768:
            value -= 65536
        return value

    def _read_accel_g(self):
        """Raw accelerometer reading in g's, one float per axis."""
        ax = self._read_raw(self.ACCEL_XOUT_H) / self.ACCEL_SCALE
        ay = self._read_raw(self.ACCEL_YOUT_H) / self.ACCEL_SCALE
        az = self._read_raw(self.ACCEL_ZOUT_H) / self.ACCEL_SCALE
        return ax, ay, az

    def _calibrate(self, samples):
        """
        Averages the accelerometer readings while the IMU sits flat and
        still. x and y should read 0g at rest, so their averages are
        pure bias. z reads +1g at rest from gravity, so that whole
        reading, gravity included, is stored as z's bias and subtracted
        out later against each live reading, the same way x and y are
        already handled.
        """
        print("Calibrating IMU, keep it flat and still...")
        sum_x = sum_y = sum_z = 0.0
        for _ in range(samples):
            ax, ay, az = self._read_accel_g()
            sum_x += ax
            sum_y += ay
            sum_z += az
            time.sleep(0.001)

        bias_x, bias_y, bias_z = sum_x / samples, sum_y / samples, sum_z / samples
        print(f"IMU calibration done. Bias  x={bias_x:.4f}g  y={bias_y:.4f}g  z={bias_z:.4f}g")
        return bias_x, bias_y, bias_z

    def read(self):
        """One fresh reading: bias-corrected acceleration in m/s^2 on
        each axis, plus roll and pitch in degrees."""
        ax_g, ay_g, az_g = self._read_accel_g()

        ax = (ax_g - self.bias_x) * self.G
        ay = (ay_g - self.bias_y) * self.G
        az = (az_g - self.bias_z) * self.G

        # roll/pitch need gravity still present, so only x and y get
        # their small sensor bias removed here, z keeps its full reading
        roll = math.degrees(math.atan2(-(ax_g - self.bias_x), az_g))
        pitch = math.degrees(math.atan2(ay_g - self.bias_y, az_g))

        return {
            "accel_x": ax,
            "accel_y": ay,
            "accel_z": az,
            "roll": roll,
            "pitch": pitch,
        }