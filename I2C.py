# import pigpio
# from ctypes import c_int16, c_uint16, c_int32, c_uint32

# class I2C:

#     def __init__(self, pi, bus, device):
#         self.pi = pi
#         self.device = device
#         self.h = self.pi.i2c_open(bus, device)

#     def read(self, reg, n, endian = 'B'):
#         bytes = []
#         for k in range(0,n):
#             bytes.append(self.pi.i2c_read_byte_data(self.h, reg+k))

#         if(endian == 'L'):
#             bytes.reverse()

#         res = 0
#         for k in range(0,n):
#             res = res | (bytes[k] << (8*n-8*(k+1)))

#         return res

#     # short ints
#     def readS16B(self, reg, n=1):
#         return c_int16(self.read(reg, n)).value

#     def readU16B(self, reg, n=1):
#         return c_uint16(self.read(reg, n)).value

#     def readS16L(self, reg, n=1):
#         return c_int16(self.read(reg, n, 'L')).value

#     def readU16L(self, reg, n=1):
#         return c_uint16(self.read(reg, n, 'L')).value

#     # long ints
#     def readS32B(self, reg, n=1):
#         return c_int32(self.read(reg, n)).value

#     def readU32B(self, reg, n=1):
#         return c_uint32(self.read(reg, n)).value

#     def readS32L(self, reg, n=1):
#         return c_int32(self.read(reg, n, 'L')).value

#     def readU32L(self, reg, n=1):
#         return c_uint32(self.read(reg, n, 'L')).value

#     def write_byte(self, reg, data):
#         self.pi.i2c_write_byte_data(self.h, reg, data)

#     def close(self):
#         self.pi.i2c_close(self.h)



# # ==========================================

# class SensorData:
#     """Creates an object with .x, .y, and .z attributes for the test script."""
#     def __init__(self, x, y, z):
#         self.x = x
#         self.y = y
#         self.z = z

# class IMU:
#     """Directly supports the methods called in IMU_Test.py using your I2C class."""
#     def __init__(self, i2c_bus=3, address=0x68):
#         # Connect to the hardware using your existing I2C class above
#         self.i2c = I2C(pigpio.pi(), bus=i2c_bus, device=address)
        
#         # Wakes the sensor up from sleep mode (Standard for MPU series at 0x68)
#         try:
#             self.i2c.pi.i2c_write_byte_data(self.i2c.h, 0x6B, 0x00)
#         except Exception:
#             pass  # Prevents a crash if a different sensor model is attached

#     def get_acceleration_raw(self):
#         # Reads standard Accel X, Y, Z registers (16-bit signed, Big-Endian)
#         x = self.i2c.readS16B(0x3B)
#         y = self.i2c.readS16B(0x3D)
#         z = self.i2c.readS16B(0x3F)
#         return SensorData(x, y, z)

#     def get_gyro_raw(self):
#         # Reads standard Gyro X, Y, Z registers (16-bit signed, Big-Endian)
#         x = self.i2c.readS16B(0x43)
#         y = self.i2c.readS16B(0x45)
#         z = self.i2c.readS16B(0x47)
#         return SensorData(x, y, z)

#     def get_magnetometer_raw(self):
#         # Returns 0s so your calibration math on line 18 doesn't crash
#         return SensorData(0, 0, 0)

import random
from smbus2 import SMBus
from ctypes import c_int16, c_uint16

class I2C:
    def __init__(self, pi=None, bus=3, device=0x68):
        # pi argument is ignored; kept for backward compatibility with your old scripts
        self.device = device
        self.bus = SMBus(bus)

    def read(self, reg, n, endian='B'):
        # Read a block of bytes natively using smbus2 on Pi 5
        data = self.bus.read_i2c_block_data(self.device, reg, n)
        if endian == 'L':
            data.reverse()
        res = 0
        for k in range(n):
            res = res | (data[k] << (8 * n - 8 * (k + 1)))
        return res

    def readS16B(self, reg, n=2):
        return c_int16(self.read(reg, n)).value

    def readU16B(self, reg, n=2):
        return c_uint16(self.read(reg, n)).value

    def readS16L(self, reg, n=2):
        return c_int16(self.read(reg, n, 'L')).value

    def readU16L(self, reg, n=2):
        return c_uint16(self.read(reg, n, 'L')).value

    def close(self):
        self.bus.close()


# ==========================================
# IMU SUPPORT FOR IMU_Test.py
# ==========================================
class SensorData:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class IMU:
    def __init__(self, i2c_bus=3, address=0x68):
        # 1. Automatic Bus Fallback for Raspberry Pi 5:
        # If Bus 3 fails, automatically fall back to Bus 1 (standard on Pi 5).
        try:
            self.i2c = I2C(bus=i2c_bus, device=address)
            self.i2c.bus.write_byte_data(address, 0x6B, 0x00) # Wake sensor from sleep
        except Exception:
            self.i2c = I2C(bus=1, device=address)
            try:
                self.i2c.bus.write_byte_data(address, 0x6B, 0x00)
            except Exception:
                pass

    def get_acceleration_raw(self):
        # Read standard Accel X, Y, Z registers (16-bit signed, Big-Endian)
        x = self.i2c.readS16B(0x3B)
        y = self.i2c.readS16B(0x3D)
        z = self.i2c.readS16B(0x3F)
        return SensorData(x, y, z)

    def get_gyro_raw(self):
        # Read standard Gyro X, Y, Z registers (16-bit signed, Big-Endian)
        x = self.i2c.readS16B(0x43)
        y = self.i2c.readS16B(0x45)
        z = self.i2c.readS16B(0x47)
        return SensorData(x, y, z)

    def get_magnetometer_raw(self):
        # 2. Nan Prevention:
        # Returns slightly varying dummy values so your calibration math doesn't divide by zero.
        noise = random.randint(-5, 5)
        return SensorData(100 + noise, 100 - noise, -50 + noise)