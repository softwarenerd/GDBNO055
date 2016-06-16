import logging
import sys
import time
import math
from Adafruit_BNO055 import BNO055

bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)

# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

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

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

print('Reading BNO055 data, press Ctrl-C to quit...')
while True:
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = bno.read_euler()

    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()

    # Gyroscope data (in degrees per second):
    gx, gy, gz = bno.read_gyroscope()

    # Accelerometer data (in meters per second squared):
    ax, ay, az = bno.read_accelerometer()

    # Read magnetometer data (in micro-Teslas).
    mx,my,mz = bno.read_magnetometer()

    # Logging.
    print('')
    print('-------------------------------------------------------------------------')
    print('BNO055 Raw Data')
    print('-------------------------------------------------------------------------')
    print('     Gyroscope (rad/s): X: {0:+0.5F}, Y: {0:+0.5F}, Z: {0:+0.5F}'.format(gx * math.pi / 180.0, gy * math.pi / 180.0, gz * math.pi / 180.0))
    print('     Gyroscope (deg/s): X: {0:+0.5F}, Y: {0:+0.5F}, Z: {0:+0.5F}'.format(gx, gy, gz))
    print('     Accelerometer (g): X: {0:+0.5F}, Y: {0:+0.5F}, Z: {0:+0.5F}'.format(ax, ay, az))
    print('     Magnetometer (uT): X: {0:+0.5F}, Y: {0:+0.5F}, Z: {0:+0.5F}'.format(mx, my, mz))

    print('-------------------------------------------------------------------------')
    print('Madgwick / BNO055 Comparison')
    print('-------------------------------------------------------------------------')
    print('      Madgwick Roll (deg): {0:+0.5F}'.format(0.0))
    print('        BNO055 Roll (deg): {0:+0.5F}'.format(roll))
    print('---------------------------------------')
    print('     Madgwick Pitch (deg): {0:+0.5F}'.format(0.0))
    print('       BNO055 Pitch (deg): {0:+0.5F}'.format(pitch))
    print('---------------------------------------')
    print('       Madgwick Yaw (deg): {0:+0.5F}'.format(0.0))
    print('         BNO055 Yaw (deg): {0:+0.5F}'.format(yaw))

    # Other values you can optionally read:
    # Orientation as a quaternion:
    #x,y,z,w = bno.read_quaterion()

    # Linear acceleration data (i.e. acceleration from movement, not gravity--
    # returned in meters per second squared):
    #x,y,z = bno.read_linear_acceleration()

    # Gravity acceleration data (i.e. acceleration just from gravity--returned
    # in meters per second squared):    
    #x,y,z = bno.read_gravity()

    time.sleep(0.2)
