import board
import busio
import adafruit_bno055

# Initialize I2C connection
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Read sensor data
print(f"Temperature: {sensor.temperature}°C")
print(f"Euler angles: {sensor.euler}")
print(f"Acceleration: {sensor.acceleration}")
print(f"Magnetic: {sensor.magnetic}")
