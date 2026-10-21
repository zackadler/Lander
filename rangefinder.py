######################################################
# Copyright (c) 2021 Maker Portal LLC
# Author: Joshua Hrisko
# Modified for stability on process interruptions
######################################################
#
# TF-Luna Mini LiDAR wired to a Raspberry Pi via UART
# --- Configuring the TF-Luna's baudrate, sample rate,
# --- and printing out the device version info 
#
######################################################
import serial
import time
import numpy as np
import sys

############################
# Serial Functions
############################

def read_tfluna_data():
    while True:
        counter = ser.in_waiting # count the number of bytes of the serial port
        bytes_to_read = 9
        if counter > bytes_to_read-1:
            bytes_serial = ser.read(bytes_to_read) # read 9 bytes
            ser.reset_input_buffer() # reset buffer

            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # check first two bytes
                distance = bytes_serial[2] + bytes_serial[3]*256 # distance in next two bytes
                strength = bytes_serial[4] + bytes_serial[5]*256 # signal strength in next two bytes
                temperature = bytes_serial[6] + bytes_serial[7]*256 # temp in next two bytes
                temperature = (temperature/8) - 256 # temp scaling and offset
                return distance/100.0, strength, temperature

def set_samp_rate(samp_rate=100):
    ##########################
    # change the sample rate
    if ser is None:
        print("Error: Serial port is not initialized.")
        return
    samp_rate_packet = [0x5a, 0x06, 0x03, samp_rate, 00, 00] # sample rate byte array
    ser.write(samp_rate_packet) # send sample rate instruction
    time.sleep(0.1) # wait for change to take effect
    return
            
def get_version():
    ##########################
    # get version info
    if ser is None:
        return
    info_packet = [0x5a, 0x04, 0x14, 0x00]

    ser.write(info_packet)
    time.sleep(0.1)
    bytes_to_read = 30
    t0 = time.time()
    while (time.time()-t0) < 5:
        counter = ser.in_waiting
        if counter > bytes_to_read:
            bytes_data = ser.read(bytes_to_read)
            ser.reset_input_buffer()
            if bytes_data[0] == 0x5a:
                version = bytes_data[3:-1].decode('utf-8', errors='ignore')
                print('Version - ' + version)
                return
            else:
                ser.write(info_packet)
                time.sleep(0.1)
    print("Timed out waiting for version info.")

def set_baudrate(baud_indx=4):
    ##########################
    # change baud rate
    baud_hex = [[0x80, 0x25, 0x00], # 9600
                [0x00, 0x4b, 0x00], # 19200
                [0x00, 0x96, 0x00], # 38400
                [0x00, 0xe1, 0x00], # 57600
                [0x00, 0xc2, 0x01], # 115200
                [0x00, 0x84, 0x03], # 230400
                [0x00, 0x08, 0x07], # 460800
                [0x00, 0x10, 0x0e]]  # 921600
    info_packet = [0x5a, 0x08, 0x06, baud_hex[baud_indx][0], baud_hex[baud_indx][1],
                   baud_hex[baud_indx][2], 0x00, 0x00] # instruction packet 

    prev_ser.write(info_packet) # change the baud rate
    time.sleep(0.1) # wait to settle
    prev_ser.close() # close old serial port
    time.sleep(0.1) # wait to settle
    
    ser_new = serial.Serial("/dev/serial0", baudrates[baud_indx], timeout=0) # new serial device
    if not ser_new.is_open:
        ser_new.open() # open serial port if not open
        
    bytes_to_read = 8
    t0 = time.time()
    while (time.time()-t0) < 3:
        counter = ser_new.in_waiting
        if counter > bytes_to_read:
            bytes_data = ser_new.read(bytes_to_read)
            ser_new.reset_input_buffer()
            if bytes_data[0] == 0x5a:
                indx = [ii for ii in range(0, len(baud_hex)) if \
                        baud_hex[ii][0] == bytes_data[3] and
                        baud_hex[ii][1] == bytes_data[4] and
                        baud_hex[ii][2] == bytes_data[5]]
                if indx:
                    print('Baud Rate verified = {0:1d}'.format(baudrates[indx[0]]))
                time.sleep(0.1) 
                return ser_new
            else:
                ser_new.write(info_packet) # try again if wrong data received
                time.sleep(0.1) 
                continue
                
    print("Baudrate handshaking timed out. Returning port anyway.")
    return ser_new

#
############################
# Configurations
############################
#
baudrates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600] 
prev_indx = 4 # Assumes TF-Luna defaults or left off at 115200

try:
    prev_ser = serial.Serial("/dev/serial0", baudrates[prev_indx], timeout=0)
    if not prev_ser.is_open:
        prev_ser.open()
except serial.SerialException as e:
    print(f"Failed to open /dev/serial0: {e}")
    print("If the port is locked, try running: sudo fuser -k /dev/serial0")
    sys.exit(1)

baud_indx = 4 
ser = set_baudrate(baud_indx) 

# Ensure ser was successfully generated
if ser is None:
    print("Fatal Error: Serial port initialization returned None.")
    sys.exit(1)

set_samp_rate(100) # set sample rate 1-250
get_version() # print version info for TF-Luna
time.sleep(0.1) 

#
############################
# Testing the TF-Luna Output
############################
#
tot_pts = 100 
t0 = time.time() 
dist_array = [] 

print("Starting data acquisition loop... Press Ctrl+C to stop.")
try:
    while len(dist_array) < tot_pts:
        try:
            distance, strength, temperature = read_tfluna_data() 
            dist_array.append(distance) 
        except Exception:
            continue
    print('Sample Rate: {0:2.0f} Hz'.format(len(dist_array)/(time.time()-t0))) 

except KeyboardInterrupt:
    print("\nExecution interrupted by user. Cleaning up...")

finally:
    # This block guarantees the port closes cleanly even if you Ctrl+C
    if 'ser' in locals() and ser is not None and ser.is_open:
        ser.close()
        print("Serial port closed cleanly.")