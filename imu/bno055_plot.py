import time
import board
import adafruit_bno055
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Initialize BNO055 sensor over I2C
i2c = board.I2C()  
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Store history of last 100 readings
data_size = 100  
accel_x = np.zeros(data_size)
accel_y = np.zeros(data_size)
accel_z = np.zeros(data_size)
gyro_x = np.zeros(data_size)
gyro_y = np.zeros(data_size)
gyro_z = np.zeros(data_size)
euler_x = np.zeros(data_size)
euler_y = np.zeros(data_size)
euler_z = np.zeros(data_size)

# Setup Matplotlib figure
fig, axes = plt.subplots(3, 1, figsize=(8, 10))

# Labels for subplots
titles = ["Acceleration (m/s²)", "Gyroscope (rad/s)", "Euler Angles (°)"]
lines = []

# Initialize plots
for ax, title in zip(axes, titles):
    ax.set_title(title)
    ax.set_ylim(-10, 10)  # Adjust y-limits if needed
    ax.set_xlim(0, data_size)
    ax.grid()
    line_x, = ax.plot(accel_x, label="X")
    line_y, = ax.plot(accel_y, label="Y")
    line_z, = ax.plot(accel_z, label="Z")
    lines.append((line_x, line_y, line_z))
    ax.legend()

# Update function for animation
def update(frame):
    global accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, euler_x, euler_y, euler_z

    # Read sensor data
    accel = sensor.acceleration or (0, 0, 0)
    gyro = sensor.gyro or (0, 0, 0)
    euler = sensor.euler or (0, 0, 0)

    # Shift data left and append new values
    accel_x = np.roll(accel_x, -1)
    accel_y = np.roll(accel_y, -1)
    accel_z = np.roll(accel_z, -1)
    accel_x[-1], accel_y[-1], accel_z[-1] = accel

    gyro_x = np.roll(gyro_x, -1)
    gyro_y = np.roll(gyro_y, -1)
    gyro_z = np.roll(gyro_z, -1)
    gyro_x[-1], gyro_y[-1], gyro_z[-1] = gyro

    euler_x = np.roll(euler_x, -1)
    euler_y = np.roll(euler_y, -1)
    euler_z = np.roll(euler_z, -1)
    euler_x[-1], euler_y[-1], euler_z[-1] = euler

    # Update plot lines
    for line, data in zip(lines[0], [accel_x, accel_y, accel_z]):
        line.set_ydata(data)
    for line, data in zip(lines[1], [gyro_x, gyro_y, gyro_z]):
        line.set_ydata(data)
    for line, data in zip(lines[2], [euler_x, euler_y, euler_z]):
        line.set_ydata(data)

    return lines

# Create animation
ani = animation.FuncAnimation(fig, update, interval=100)

# Show plot
plt.tight_layout()
plt.show()
