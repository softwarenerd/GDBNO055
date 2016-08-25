import logging
import sys
import time
import math
from Adafruit_BNO055 import BNO055
import warnings
import numpy as np
from numpy.linalg import norm
import numbers

def degreesToRadians(degrees)
    return degrees * math.pi / 180.0

def radiansToDegrees(float radians)
    return radians * 180.0 / math.pi

def main():
    # Altert user.
    print('Starting...')

    # Allocate the driver.
    bno = BNO055.BNO055(serial_port='/dev/ttyS0', rst=18)

    # Initialize the BNO055 and stop if something went wrong.
    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

    # Print system status and self test result.
    status, self_test, error = bno.get_system_status()
    print('System status: {0}'.format(status))
    print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))

    # Print out an error if system status is in error mode.
    if status == 0x01:
        print('System error: {0}'.format(error))
        print('See datasheet section 4.3.59 for the meaning.')
        raise RuntimeError('BNO055 system error.')

    # Print BNO055 software revision and other diagnostic data.
    sw, bl, accel, mag, gyro = bno.get_revision()
    print('Software version:   {0}'.format(sw))
    print('Bootloader version: {0}'.format(bl))
    print('Accelerometer ID:   0x{0:02X}'.format(accel))
    print('Magnetometer ID:    0x{0:02X}'.format(mag))
    print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

    print('Reading BNO055 data, press Ctrl-C to quit...')
    count = 1
    while True:
        # Read the Euler angles for yaw, roll, pitch (all in degrees).
        yaw, roll, pitch = bno.read_euler()

        # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
        sys, gyro, accel, mag = bno.get_calibration_status()

        # Gyroscope data (in degrees per second):
        gx_deg_sec, gy_deg_sec, gz_deg_sec = bno.read_gyroscope()

        # Convert gyroscope data from degrees per second to radians per second. 
        gx_rad_sec = degreesToRadians(gx_deg_sec)
        gy_rad_sec = degreesToRadians(gy_deg_sec)
        gz_rad_sec = degreesToRadians(gz_deg_sec)

        # Accelerometer data (in meters per second squared):
        ax, ay, az = bno.read_accelerometer()

        # Read magnetometer data (in micro-Teslas).
        mx, my, mz = bno.read_magnetometer()

        madgwick.update([gx, gy, gz], [ax, ay, az], [mx, my, mz])
        madgwickRoll, madgwickPitch, madgwickYaw = madgwick.quaternion.to_euler_angles()
        madgwickRoll = madgwickRoll * 180.0 / math.pi
        madgwickPitch = madgwickPitch * 180.0 / math.pi
        madgwickYaw = madgwickYaw * 180.0 / math.pi

        print('-------------------------------------------------------------------------')
        print('Sample {0}'.format(count))
        count += 1
        print('       System Calibration={0}'.format(sys))
        print('    Gyroscope Calibration={0}'.format(gyro))
        print('Accelerometer Calibration={0}'.format(accel))
        print(' Magnetometer Calibration={0}'.format(mag))

        # Logging.
        if sys == 3:
            print('-------------------------------------------------------------------------')
            print('     Gyroscope (rad/s): X: {0:+0.5F}, Y: {1:+0.5F}, Z: {2:+0.5F}'.format(gx_rad_sec, gy_rad_sec, gz_rad_sec))
            print('     Gyroscope (deg/s): X: {0:+0.5F}, Y: {1:+0.5F}, Z: {2:+0.5F}'.format(gx_deg_sec, gy_deg_sec, gz_deg_sec))
            print('     Accelerometer (g): X: {0:+0.5F}, Y: {1:+0.5F}, Z: {2:+0.5F}'.format(ax, ay, az))
            print('     Magnetometer (uT): X: {0:+0.5F}, Y: {1:+0.5F}, Z: {2:+0.5F}'.format(mx, my, mz))
            print('            Total mag: ')
            print('-------------------------------------------------------------------------')
            print('BNO055 Euler')
            print('-------------------------------------------------------------------------')
            print('        BNO055 Roll (deg): {0:+0.5F}'.format(roll))
            print('---------------------------------------')
            print('       BNO055 Pitch (deg): {0:+0.5F}'.format(pitch))
            print('---------------------------------------')
            print('         BNO055 Yaw (deg): {0:+0.5F}'.format(yaw))
        else:
            print("BNO055 Raw Data - UNCALIBRATED")



# Execute main.
if __name__ == '__main__':
    main()