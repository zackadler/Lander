import smbus					#import SMBus module of I2C
from time import sleep          #import

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
	
	#Write to power management register
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
		acc_x = read_Offset_data(ACCEL_XOUT_H)
		acc_y = read_Offset_data(ACCEL_YOUT_H)
		acc_z = read_Offset_data(ACCEL_ZOUT_H)
		gyro_x = read_Offset_data(GYRO_XOUT_H)
		gyro_y = read_Offset_data(GYRO_YOUT_H)
		gyro_z = read_Offset_data(GYRO_ZOUT_H)
		Offset_acc_x += acc_x
		Offset_acc_y += acc_y
		Offset_acc_z += acc_z
		Offset_gyro_x += gyro_x
		Offset_gyro_y += gyro_y
		Offset_gyro_z += gyro_z
		i = i+1

	Offset_acc_x = Offset_acc_x/data_pts
	Offset_acc_y = Offset_acc_y/data_pts
	Offset_acc_z = Offset_acc_z/data_pts
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
def read_Offset_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address
Offset_acc_x, Offset_acc_y, Offset_acc_z, Offset_gyro_x, Offset_gyro_y, Offset_gyro_z = MPU_Init()

print (" Reading Data of Gyroscope and Accelerometer")

while True:
	
	#Read Accelerometer Offset value
	acc_x = read_Offset_data(ACCEL_XOUT_H)
	acc_y = read_Offset_data(ACCEL_YOUT_H)
	acc_z = read_Offset_data(ACCEL_ZOUT_H)
	
	#Read Gyroscope Offset value
	gyro_x = read_Offset_data(GYRO_XOUT_H)
	gyro_y = read_Offset_data(GYRO_YOUT_H)
	gyro_z = read_Offset_data(GYRO_ZOUT_H)
	
	#Full scale range +/- 250 degree/C as per sensitivity scale factor
	Ax = acc_x/16384.0 -  Offset_acc_x/16384.0 
	Ay = acc_y/16384.0- Offset_acc_y/16384.0
	Az = acc_z/16384.0- Offset_acc_z/16384.0
	
	Gx = gyro_x/131.0 -  Offset_gyro_x/131.0
	Gy = gyro_y/131.0  -  Offset_gyro_y/131.0
	Gz = gyro_z/131.0 -  Offset_gyro_z/131.0
	

	print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s","\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
	sleep(1)
