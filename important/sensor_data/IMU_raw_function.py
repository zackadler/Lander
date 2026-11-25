"""
Raw ICM-20948 driver for data gathering and EKF input.

Differences from the old ImuSensor:
  * Reads accel + gyro + temperature in ONE 14-byte burst (no torn
    high/low byte reads, and all channels are time-coherent).
  * Exposes the gyroscope and (via I2C bypass) the AK09916 magnetometer.
  * NO calibration or bias removal: returns raw values in SI units, so
    you can calibrate yourself in post-processing.
  * Configurable ranges / digital low-pass filters / sample-rate divider.
  * Checks WHO_AM_I at start-up (expect 0xEA).

Axes are the chip's NATIVE axes. The old driver's docstring said:
x = right, y = forward, z = up. Verify with the six-position test, then
use to_frd() below to map into FRD (x Forward, y Right, z Down).
NOTE: on the ICM-20948 the magnetometer axes are NOT necessarily aligned
with the accel/gyro axes. Don't assume; check by rotating about known axes.

Register addresses are from the ICM-20948 / AK09916 datasheets. If a
channel looks dead, check WHO_AM_I and the register map first.
"""

import struct
import time

import smbus

G = 9.80665
DEG2RAD = 0.017453292519943295

# Native chip axes -> FRD. Rows are the FRD x, y, z expressed in native axes,
# ASSUMING native x=right, y=forward, z=up. This is a proper rotation (det=+1).
# VERIFY with the six-position test before trusting it.
NATIVE_TO_FRD = (
    (0, 1, 0),    # FRD x (forward) =  native y
    (1, 0, 0),    # FRD y (right)   =  native x
    (0, 0, -1),   # FRD z (down)    = -native z
)


def to_frd(v):
    """Map a native-axis 3-vector (x, y, z) into FRD."""
    return tuple(sum(r[i] * v[i] for i in range(3)) for r in NATIVE_TO_FRD)


class Icm20948Raw:
    # --- register bank select ---
    REG_BANK_SEL = 0x7F

    # --- bank 0 ---
    WHO_AM_I = 0x00
    USER_CTRL = 0x03
    PWR_MGMT_1 = 0x06
    PWR_MGMT_2 = 0x07
    INT_PIN_CFG = 0x0F
    ACCEL_XOUT_H = 0x2D        # 14 bytes: accel(6) gyro(6) temp(2)

    # --- bank 2 ---
    GYRO_SMPLRT_DIV = 0x00
    GYRO_CONFIG_1 = 0x01
    ACCEL_SMPLRT_DIV_1 = 0x10
    ACCEL_SMPLRT_DIV_2 = 0x11
    ACCEL_CONFIG = 0x14

    WHO_AM_I_EXPECTED = 0xEA

    # --- AK09916 magnetometer (on the main bus via bypass) ---
    MAG_ADDR = 0x0C
    MAG_WIA1 = 0x00            # expect 0x48
    MAG_WIA2 = 0x01            # expect 0x09
    MAG_ST1 = 0x10             # ST1, HXL..HZH, TMPS, ST2 = 9 bytes
    MAG_CNTL2 = 0x31
    MAG_CNTL3 = 0x32
    MAG_UT_PER_LSB = 0.15

    ACCEL_LSB_PER_G = {0: 16384.0, 1: 8192.0, 2: 4096.0, 3: 2048.0}   # +/-2,4,8,16 g
    GYRO_LSB_PER_DPS = {0: 131.0, 1: 65.5, 2: 32.8, 3: 16.4}          # +/-250,500,1000,2000 dps

    def __init__(self, bus_num=3, address=0x68,
                 accel_fs=1, gyro_fs=1,
                 accel_dlpf=3, gyro_dlpf=3,
                 rate_div=4, use_mag=True):
        """
        accel_fs / gyro_fs : full-scale index (accel 0..3 = 2/4/8/16 g,
                             gyro 0..3 = 250/500/1000/2000 dps).
        accel_dlpf/gyro_dlpf: DLPF config 0..7. 3 is ~50 Hz bandwidth,
                             2 is ~110-120 Hz. 0..7 per datasheet table.
        rate_div           : output rate = 1125 Hz / (1 + rate_div)
                             (4 -> ~225 Hz).
        """
        self.address = address
        self.bus = smbus.SMBus(bus_num)
        self.accel_scale = self.ACCEL_LSB_PER_G[accel_fs]
        self.gyro_scale = self.GYRO_LSB_PER_DPS[gyro_fs]
        self.has_mag = False
        self._mag_cache = (float("nan"),) * 3
        self._mag_ovf = 0

        self._select_bank(0)
        who = self.bus.read_byte_data(self.address, self.WHO_AM_I)
        self.who_am_i = who
        if who != self.WHO_AM_I_EXPECTED:
            raise RuntimeError(
                f"WHO_AM_I = 0x{who:02X}, expected 0x{self.WHO_AM_I_EXPECTED:02X} "
                f"(ICM-20948). Wrong chip or address?"
            )

        self._init_chip(accel_fs, gyro_fs, accel_dlpf, gyro_dlpf, rate_div)
        if use_mag:
            self._init_mag()

    # ------------------------------------------------------------------
    def _select_bank(self, bank):
        self.bus.write_byte_data(self.address, self.REG_BANK_SEL, (bank & 0x03) << 4)

    def _init_chip(self, accel_fs, gyro_fs, accel_dlpf, gyro_dlpf, rate_div):
        a = self.address
        self._select_bank(0)
        self.bus.write_byte_data(a, self.PWR_MGMT_1, 0x80)   # device reset
        time.sleep(0.1)
        self.bus.write_byte_data(a, self.PWR_MGMT_1, 0x01)   # wake, auto clock
        time.sleep(0.05)
        self.bus.write_byte_data(a, self.PWR_MGMT_2, 0x00)   # enable accel + gyro
        time.sleep(0.05)

        self._select_bank(2)
        self.bus.write_byte_data(a, self.GYRO_SMPLRT_DIV, rate_div & 0xFF)
        self.bus.write_byte_data(a, self.GYRO_CONFIG_1,
                                 ((gyro_dlpf & 7) << 3) | ((gyro_fs & 3) << 1) | 1)
        self.bus.write_byte_data(a, self.ACCEL_SMPLRT_DIV_1, (rate_div >> 8) & 0x0F)
        self.bus.write_byte_data(a, self.ACCEL_SMPLRT_DIV_2, rate_div & 0xFF)
        self.bus.write_byte_data(a, self.ACCEL_CONFIG,
                                 ((accel_dlpf & 7) << 3) | ((accel_fs & 3) << 1) | 1)

        self._select_bank(0)
        time.sleep(0.1)

    def _init_mag(self):
        a = self.address
        self._select_bank(0)
        self.bus.write_byte_data(a, self.USER_CTRL, 0x00)     # internal I2C master off
        self.bus.write_byte_data(a, self.INT_PIN_CFG, 0x02)   # BYPASS_EN: mag on main bus
        time.sleep(0.01)
        try:
            wia1 = self.bus.read_byte_data(self.MAG_ADDR, self.MAG_WIA1)
            wia2 = self.bus.read_byte_data(self.MAG_ADDR, self.MAG_WIA2)
        except OSError:
            self.has_mag = False
            return
        self.mag_ids = (wia1, wia2)
        self.has_mag = (wia1 == 0x48 and wia2 == 0x09)
        if self.has_mag:
            self.bus.write_byte_data(self.MAG_ADDR, self.MAG_CNTL3, 0x01)  # soft reset
            time.sleep(0.1)
            self.bus.write_byte_data(self.MAG_ADDR, self.MAG_CNTL2, 0x08)  # 100 Hz continuous

    # ------------------------------------------------------------------
    def read(self):
        """One burst read. Accel in m/s^2, gyro in rad/s, mag in uT,
        temp in deg C. No bias removal. Native chip axes."""
        data = self.bus.read_i2c_block_data(self.address, self.ACCEL_XOUT_H, 14)
        ax, ay, az, gx, gy, gz, t = struct.unpack(">7h", bytes(data))

        mag_new = 0
        if self.has_mag:
            try:
                m = self.bus.read_i2c_block_data(self.MAG_ADDR, self.MAG_ST1, 9)
                if m[0] & 0x01:   # DRDY
                    mx, my, mz = struct.unpack("<3h", bytes(m[1:7]))
                    self._mag_cache = (mx * self.MAG_UT_PER_LSB,
                                       my * self.MAG_UT_PER_LSB,
                                       mz * self.MAG_UT_PER_LSB)
                    self._mag_ovf = (m[8] >> 3) & 1
                    mag_new = 1
            except OSError:
                pass

        k = G / self.accel_scale
        w = DEG2RAD / self.gyro_scale
        return {
            "accel_x": ax * k, "accel_y": ay * k, "accel_z": az * k,
            "gyro_x": gx * w, "gyro_y": gy * w, "gyro_z": gz * w,
            "temp_c": t / 333.87 + 21.0,
            "mag_x": self._mag_cache[0], "mag_y": self._mag_cache[1],
            "mag_z": self._mag_cache[2],
            "mag_new": mag_new, "mag_ovf": self._mag_ovf,
            "raw_ax": ax, "raw_ay": ay, "raw_az": az,
            "raw_gx": gx, "raw_gy": gy, "raw_gz": gz,
        }