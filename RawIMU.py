import logging
import sys
import time
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
    print('-------------------------------------------------------------------------'')
    print(@"BNO055 Raw Data");
    print('-------------------------------------------------------------------------'')
    print('     Gyroscope (rad/s): X: {0:0.5F}, Y: {0:0.5F}, Z: {0:0.5F}, [_numberFormatterLogging stringFromNumber:@(gx)'.format(gx, gy, gz))

    #NSLog(@"     Gyroscope (deg/s): X: %@, Y: %@, Z: %@", [_numberFormatterLogging stringFromNumber:@(RadiansToDegrees(gx))],
    #                                                      [_numberFormatterLogging stringFromNumber:@(RadiansToDegrees(gy))],
    #                                                      [_numberFormatterLogging stringFromNumber:@(RadiansToDegrees(gz))]);
    #NSLog(@"     Accelerometer (g): X: %@, Y: %@, Z: %@", [_numberFormatterLogging stringFromNumber:@(ax * -1.0)],
    #                                                      [_numberFormatterLogging stringFromNumber:@(ay * -1.0)],
    #                                                      [_numberFormatterLogging stringFromNumber:@(az * -1.0)]);
    #NSLog(@"     Magnetometer (uT): X: %@, Y: %@, Z: %@", [_numberFormatterLogging stringFromNumber:@(mx)],
    #                                                      [_numberFormatterLogging stringFromNumber:@(my)],
    #                                                      [_numberFormatterLogging stringFromNumber:@(mz)]);
    #NSLog(@"-------------------------------------------------------------------------");
    #NSLog(@"Madgwick / CoreMotion Comparison");
    #NSLog(@"-------------------------------------------------------------------------");
    #NSLog(@"      Madgwick Roll (deg): %@", [_numberFormatterLogging stringFromNumber:@(roll)]);
    #NSLog(@"    CoreMotion Roll (deg): %@", [_numberFormatterLogging stringFromNumber:@(coreMotionRoll)]);
    #NSLog(@"---------------------------------------");
    #NSLog(@"     Madgwick Pitch (deg): %@", [_numberFormatterLogging stringFromNumber:@(pitch)]);
    #NSLog(@"   CoreMotion Pitch (deg): %@", [_numberFormatterLogging stringFromNumber:@(coreMotionPitch)]);
    #NSLog(@"---------------------------------------");
    #NSLog(@"       Madgwick Yaw (deg): %@", [_numberFormatterLogging stringFromNumber:@(yaw)]);
    #NSLog(@"     CoreMotion Yaw (deg): %@", [_numberFormatterLogging stringFromNumber:@(coreMotionYaw)]);
    
    # Print everything out.
    #print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
    #      heading, roll, pitch, sys, gyro, accel, mag))

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
