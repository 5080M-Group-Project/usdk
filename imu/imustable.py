import time
import board
import adafruit_bno055

i2c = board.I2C()
imu = adafruit_bno055.BNO055_I2C(i2c)

while True:
    euler = imu.euler
    gyro = imu.gyro

    if gyro and gyro[3] is not None:
        yawrate = gyro[3]
    else:
        yawrate = None
    # Safely handle pitch (euler[2])
    if euler and euler[2] is not None:
        if euler[2] < 0:
            pitch = -180 - euler[2]
        else:
            pitch = 180 - euler[2]
    else:
        pitch = None

    # Safely handle gyro (gyro[2])
    if gyro and gyro[2] is not None:
        pitchrate = -1 * gyro[2]
    else:
        pitchrate = None

    if pitchrate is not None:
        print(f"Gyroscope (rad/sec): {pitchrate:.2f}")


    if pitch is not None:
        print(f"Pitch angle: {pitch:.2f}")
    if yawrate is not None:
        print(f"Yawrate: {yawrate:.2f}")

    print()
    time.sleep(0.05)
