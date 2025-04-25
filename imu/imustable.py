
import time
import board
import adafruit_bno055

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bno055.BNO055_I2C(i2c)

last_val = 0xFFFF


while True:

    print("Gyroscope (rad/sec): {}".format(sensor.gyro[2]))
    print("pitch angle: {}".format(sensor.euler[2]))

    print()

