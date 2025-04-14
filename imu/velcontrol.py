import time
import math
import sys
import board
import busio
import adafruit_bno055
import pygame
import csv

# Initialize pygame
pygame.init()

# Initialize joystick
pygame.joystick.init()

# Ensure at least one joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick detected!")
    exit()

# Get the first joystick (F710 should be js0)
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Connected to: {joystick.get_name()}")

# Tracking button states
prev_axis1 = -0.00
prev_axis0 = -0.00

sys.path.append('../lib')
from unitree_actuator_sdk import *
from functionsIMU import *  # Should include getOutputAngleDeg()

# --- Setup motor ---
serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()
cmd.id = 2
cmd.motorType = MotorType.A1
data.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)

# Gain tuning
kpOutWheel, kdOutWheel = 10.4, 2.3 # 25.4 2.3
kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)

# --- Setup IMU ---
i2c = board.I2C()
imu = adafruit_bno055.BNO055_I2C(i2c)

# --- Initial motor setup ---
cmd.q = 0.0
cmd.dq = 0.0
cmd.tau = 0.0
cmd.kp = 0.0
cmd.kd = 0.0
serial.sendRecv(cmd, data)
time.sleep(0.1)

print("🟢 Pitch stabilization running...")

# Open CSV file for logging
log_filename = f"imu_motor_log{kpOutWheel:.2f},{kdOutWheel:.2f}.csv"
with open(log_filename, mode="w", newline="") as log_file:
    log_writer = csv.writer(log_file)
    log_writer.writerow(["Time (s)", "Pitch (deg)", "kp", "kd", "Correction (deg)"])

    try:
        start_time = time.time()
        current_motor_q = 0.0  # Track motor position

        while True:
            gyro = imu.gyro
            pitchrate = gyro[1] if euler else None  # Pitch in degrees
            if pitchrate is None:
                print("⚠️ IMU is not responding")
                time.sleep(0.01)
                continue

            # Compute relative correction
            correction = -math.radians(pitchrate)  # Convert degrees to radians
            #correction = max(min(correction, math.radians(90)), math.radians(-90))  # Limit correction

            # Process joystick events
            pygame.event.pump()

            # Read joystick inputs
            axis_0 = round(joystick.get_axis(0), 2)  # Left stick X-axis
            axis_1 = round(joystick.get_axis(1), 2)  # Left stick Y-axis

            # Adjust kd with dpad up/down
            if axis_1 == -1.00 and prev_axis1 == -0.00:
                kdOutWheel += 0.1
                print(f"dpad up pressed, kd increased to {kdOutWheel:.1f}")

            if axis_1 == 1.00 and prev_axis1 == -0.00:
                kdOutWheel -= 0.1
                print(f"dpad down pressed, kd decreased to {kdOutWheel:.1f}")

            # Adjust kp with dpad left/right
            if axis_0 == -1.00 and prev_axis0 == -0.00:
                kpOutWheel -= 0.1
                print(f"dpad left pressed, kp decreased to {kpOutWheel:.1f}")

            if axis_0 == 1.00 and prev_axis0 == -0.00:
                kpOutWheel += 0.1
                print(f"dpad right pressed, kp increased to {kpOutWheel:.1f}")

            # Update previous joystick states
            prev_axis1 = axis_1
            prev_axis0 = axis_0

            # Update motor position and gains
            #current_motor_q += correction  # Increment motor position
            cmd.dq = correction
            cmd.tau = 0.0
            kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)
            cmd.kp = kpRotorWheel
            cmd.kd = kdRotorWheel
            success = serial.sendRecv(cmd, data)

            # Log data
            elapsed_time = time.time() - start_time
            log_writer.writerow([elapsed_time, pitch, kpOutWheel, kdOutWheel, math.degrees(correction)])
            log_file.flush()  # Ensure data is written to the file

            # Print debug info
            print(f"Time: {elapsed_time:.2f}s | Pitch rate: {pitchrate:.2f}°/s | kp: {kpOutWheel:.1f} | kd: {kdOutWheel:.1f} | Correction: {math.degrees(correction):+.2f}°/s")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n🛑 Stopping stabilization.")
