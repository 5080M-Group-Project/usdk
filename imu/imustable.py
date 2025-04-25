
import time
import board
import adafruit_bno055

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
imu = adafruit_bno055.BNO055_I2C(i2c)

last_val = 0xFFFF


while True:
    euler = imu.euler
    gyro = imu.gyro
    if euler:
        if euler[2] < 0:
            pitch = -180-(euler[2])
        else:
            pitch = 180-(euler[2])
    else:
        pitch = None
    if gyro:
        pitchrate = 10*(gyro[2])
    else:
        pitchrate = None
    print(f"Gyroscope (rad/sec): {pitchrate:.2f}")
    print(f"pitch angle: {pitch:.2f}")

    print()

