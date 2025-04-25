
import time
import board
import adafruit_bno055

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bno055.BNO055_I2C(i2c)

last_val = 0xFFFF


while True:
    pitch = sensor.euler[2]
    pitchrate = sensor.gyro[2]
    print(f"Gyroscope (rad/sec): {pitchrate:.2f}")
    print(f"pitch angle: {pitch:.2f}")

    print()

