import time
import math
import serial
import numpy as np
from simple_pid import PID
import adafruit_bno055
import board
import busio
import sys

# Add Unitree SDK path
sys.path.append('../lib') 
from unitree_actuator_sdk import *
from functionsIMU import *  # Make sure this exists and is correctly implemented

# Initialize motor serial communication
serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()
gearRatio = queryGearRatio(MotorType.A1)

# Initialize IMU
i2c = board.I2C()
imu = adafruit_bno055.BNO055_I2C(i2c)

# Motor Setup
cmd.motorType = MotorType.A1
data.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
cmd.id = 0  

# Rotor control gains (convert from output gains)
kpOutWheel, kdOutWheel = 0.0, 0.0
kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)

# PID Controller for stabilizing pitch
kp, ki, kd = 3.0, 0.1, 0.5
pid = PID(kp, ki, kd, setpoint=0.0)  # Desired pitch is 0°
pid.output_limits = (-math.radians(10), math.radians(10))  # ±10° in radians

sleepTime = 1  # seconds

print("🟢 Starting pitch stabilization loop...")

try:
    while True:
        # Read pitch angle from IMU
        euler = imu.euler
        current_pitch = euler[1] if euler else None  # Pitch in degrees

        if current_pitch is None:
            print("⚠️ IMU data not available. Skipping this loop iteration.")
            time.sleep(sleepTime)
            continue

        # Compute PID correction (output is in radians)
        correction_rad = pid(current_pitch)

        # Read current motor position
        if serial.sendRecv(cmd, data):
            motor_angle_deg = getOutputAngleDeg(data.q)
            motor_target_deg = motor_angle_deg + math.degrees(correction_rad)
            motor_target_rad = math.radians(motor_target_deg)

            # Send corrected motor angle
            cmd.q = motor_target_rad
            cmd.kp = kpRotorWheel
            cmd.kd = kdRotorWheel
            cmd.dq = 0.0
            cmd.tau = 0.0

            serial.sendRecv(cmd, data)

            print(f"Pitch: {current_pitch:.2f}°, Motor: {motor_angle_deg:.2f}° → Target: {motor_target_deg:.2f}°")
        else:
            print("❌ Motor not responding")

        time.sleep(sleepTime)

except KeyboardInterrupt:
    print("\n🛑 YOU KNOW WHO I AM...")