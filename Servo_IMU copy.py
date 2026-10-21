import random
import smbus                    #import SMBus module of I2C
import math
from time import sleep
from adafruit_servokit import ServoKit
import time

# Servo motor
servo_roll=5
servo_pitch=4

# Initialize the ServoKit with 16 channels and the default I2C address (0x40)
kit = ServoKit(channels=16)
kit.frquency = 100






'''
        Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
    http://www.electronicwings.com
'''


Servo_roll_Offset=15
Servo_pitch_Offset=15
angle_roll_max=15
angle_roll_min=-15
angle_pitch_max=15
angle_pitch_min=-15
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    
    #Write to power managangle_roll_max=15
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    
    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)
    
    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    
    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)
    Offset_acc_x= 0
    Offset_acc_y= 0
    Offset_acc_z= 0
    Offset_gyro_x= 0
    Offset_gyro_y= 0
    Offset_gyro_z= 0
    i = 0
    data_pts = 1000
    while i < data_pts:
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)
        Offset_acc_x += acc_x
        Offset_acc_y += acc_y
        Offset_acc_z += acc_z
        Offset_gyro_x += gyro_x
        Offset_gyro_y += gyro_y
        Offset_gyro_z += gyro_z
        i = i+1

    Offset_acc_x = Offset_acc_x/data_pts
    Offset_acc_y = Offset_acc_y/data_pts
    Offset_acc_z = -16384+Offset_acc_z/data_pts
    Offset_gyro_x = Offset_gyro_x/data_pts
    Offset_gyro_y = Offset_gyro_y/data_pts
    Offset_gyro_z = Offset_gyro_z/data_pts 
    return (
        Offset_acc_x,
        Offset_acc_y,
        Offset_acc_z,
        Offset_gyro_x,
        Offset_gyro_y,
        Offset_gyro_z
    )

def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


bus = smbus.SMBus(1)     # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address
Offset_acc_x, Offset_acc_y, Offset_acc_z, Offset_gyro_x, Offset_gyro_y, Offset_gyro_z = MPU_Init()
MPU_Init()
print (" Reading Data of Gyroscope and Accelerometer")

while True:
    
    #Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    
    #Read Gyroscope raw value
#     gyro_x = read_raw_data(GYRO_XOUT_H)
#     gyro_y = read_raw_data(GYRO_YOUT_H)
#     gyro_z = read_raw_data(GYRO_ZOUT_H)
    
    #Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = acc_x/16384.0 -  Offset_acc_x/16384.0 
    Ay = acc_y/16384.0- Offset_acc_y/16384.0
    Az = acc_z/16384.0- Offset_acc_z/16384.0
    
#     Gx = gyro_x/131.0 -  Offset_gyro_x/131.0
#     Gy = gyro_y/131.0  -  Offset_gyro_y/131.0
#     Gz = gyro_z/131.0 -  Offset_gyro_z/131.0
    
    Roll=(180/math.pi)*math.atan2(Ay,(math.sqrt(Ax ** 2+Az ** 2)))
    Pitch=-(180/math.pi)*math.atan2(-Ax,(math.sqrt(Ay ** 2+Az ** 2)))
    if Roll > angle_roll_max:
        Roll=angle_roll_max
    elif Roll < angle_roll_min:
        Roll=angle_roll_min
    if Pitch > angle_pitch_max:
        Pitch=angle_pitch_max
    elif Pitch < angle_pitch_min:
        Pitch=angle_pitch_min
    # Move the Servo to that angle measured from IMU
    #kit.servo[servo_pitch].angle = Pitch+Servo_Offset
   
    Calib_roll = Roll+Servo_roll_Offset
    Calib_pitch = Pitch+Servo_roll_Offset
    kit.servo[servo_roll].angle = Calib_roll
    kit.servo[servo_pitch].angle = Calib_pitch

    
    print(
#         f"Ax={Ax:.2f}        "
#         f"Ay={Ay:.2f}        "
#         f"Az={Az:.2f}\n"
         #f"Roll Angle={Roll:.2f}            "
         #f"Pitch Angle= {Pitch:.2f} \n "
        f"Calib_roll= {Calib_roll:.2f} \n "
        f"Calib_pitch= {Calib_pitch:.2f} \n "
    )
    sleep(.01)