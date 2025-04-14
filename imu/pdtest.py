import time
import math
import sys
import board
import busio
import adafruit_bno055
from datetime import datetime
from simple_pid import PID
import csv
import pygame

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
kpOutWheel, kdOutWheel = 2, 0.5
kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)

# --- Setup IMU ---
i2c = board.I2C()
imu = adafruit_bno055.BNO055_I2C(i2c)

# --- Setup PID ---
#1.5,0,0.5
Kp, Ki, Kd = 5,0,1
pid = PID(Kp, Ki, Kd, setpoint=0.0)  # Tune these later
pid.sample_time = 0.01
pid.output_limits = (-math.radians(60), math.radians(60))  # motor command in radians

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
log_filename = f"imu_motor_log{Kp:.2f},{Ki:.2f},{Kd:.2f}.csv"
with open(log_filename, mode="w", newline="") as log_file:
    log_writer = csv.writer(log_file)
    log_writer.writerow(["Time (s)", "Pitch (deg)", "kp", "ki", "kd", "Correction (deg)"])

    try:
        start_time = time.time()
        current_motor_q = 0.0  # We accumulate changes to q here

        while True:
            euler = imu.euler
            pitch = euler[1] if euler else None  # Pitch in degrees
            #pid = PID(Kp, Ki, Kd, setpoint=0.0)
            if pitch is None:
                print("⚠️ IMU error")
                time.sleep(0.01)
                continue

            correction = pid(math.radians(pitch))  # PID returns delta q in radians

            # Apply correction to target position
            current_motor_q += correction  # This accumulates q over time
            # Process joystick events
            pygame.event.pump()

            # Read joystick inputs
            axis_0 = round(joystick.get_axis(0), 2)  # Left stick X-axis
            axis_1 = round(joystick.get_axis(1), 2)  # Left stick Y-axis

            # Adjust kd with dpad up/down
            if axis_1 == -1.00 and prev_axis1 == -0.00:
                Kp += 0.1
                print(f"dpad up pressed, kp increased to {Kp:.1f}")

            if axis_1 == 1.00 and prev_axis1 == -0.00:
                Kp -= 0.1
                print(f"dpad down pressed, kp decreased to {Kp:.1f}")

            # Adjust kp with dpad left/right
            if axis_0 == -1.00 and prev_axis0 == -0.00:
                Kd -= 0.1
                print(f"dpad left pressed, kd decreased to {Kd:.1f}")

            if axis_0 == 1.00 and prev_axis0 == -0.00:
                Kd += 0.1
                print(f"dpad right pressed, kd increased to {Kd:.1f}")

            # Update previous joystick states
            prev_axis1 = axis_1
            prev_axis0 = axis_0
            # Send motor command
            cmd.q = current_motor_q
            #cmd.dq = 0.0 #1.0 #speed motor, maybe we can control this too
            cmd.tau = 0.0
            cmd.kp = kpRotorWheel
            cmd.kd = kdRotorWheel

            serial.sendRecv(cmd, data)
            # Log data
            elapsed_time = time.time() - start_time
            log_writer.writerow([elapsed_time, pitch, Kp, Ki, Kd, math.degrees(correction)])
            log_file.flush()  # Ensure data is written to the file
            print(f"[{elapsed_time:.2f}s] Pitch: {pitch:.2f}°, Correction: {math.degrees(correction):+.2f}°, kp: {Kp:.2f}, ki: {Ki:.2f}, kd: {Kd:.2f}, Motor q: {math.degrees(current_motor_q):.2f}°")




            #time.sleep(0.01)

    except KeyboardInterrupt:
        print("stopping")
        log_file.close()
