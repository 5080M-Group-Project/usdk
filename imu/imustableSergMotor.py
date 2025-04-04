import time
import math
import serial
import numpy as np
from simple_pid import PID
import adafruit_bno055
import board
import busio
import sys
sys.path.append('../lib') 
from unitree_actuator_sdk import *
from functionsIMU import *



serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()
gearRatio = queryGearRatio(MotorType.A1)
# Initialize I2C for the BNO055 IMU
i2c = board.I2C()
imu = adafruit_bno055.BNO055_I2C(i2c)

cmd.motorType = MotorType.A1
data.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)



# Initialize serial communication with the motor
  # Adjust as needed
baud_rate = 115200

# PID Controller for position control
# Initialize wheel Motor
kp, ki, kd = 3.0, 0.1, 0.5  # Tune these gains as needed
angleDesired = 0.0
kpOutWheel, kdOutWheel = 10.0, 3.0 ### IDEA: Modify throughout the loop i.e. when locking legs
kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)

wheelOutputAngleCurrent = 0.0

#Initialize Loop Variables
torque = 0.0
wheelOutputAngularVelocityDesired, wheelRotorAngularVelocityDesired = 0.0, 0.0
tau = 0.0
sleepTime = 0.1
pid = PID(kp, ki, kd, setpoint=0)
pid.output_limits = (-math.radians(10), math.radians(10))  # Limit output to ±10 degrees
cmd.id = 0

motor_angle_initial_deg = getOutputAngleDeg(data.q)  # Get current motor angle in degrees


while True:
    euler = imu.euler
    print(f"IMU Euler: {euler}")

    current_pitch = euler[1]
    if current_pitch is None:
        print("⚠️ IMU data not available. Skipping iteration.")
        time.sleep(0.01)
        continue

    # Run PID
    pitch_error = pid(current_pitch)

    # Read motor angle
    if serial.sendRecv(cmd, data):
        motor_current_deg = getOutputAngleDeg(data.q)
        motor_target_deg = motor_current_deg + math.degrees(pitch_error)
        motor_target_rad = math.radians(motor_target_deg)

        cmd.q = motor_target_rad
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel
        cmd.dq = 0.0
        cmd.tau = 0.0

        serial.sendRecv(cmd, data)
    else:
        print("[WARNING] Motor not responding")

    time.sleep(0.01)

'''''
while True:
    # Get current pitch angle from the IMU
    print(f"IMU euler output: {imu.euler}")
    current_pitch = imu.euler[1]  # Assuming this function returns the pitch in degrees
    current_pitch = imu.euler[1]
    if current_pitch is None:
      print("⚠️ IMU data not available. Skipping this loop iteration.")
      time.sleep(0.01)
      continue
    

    # Calculate the error (difference between desired and current pitch)
    pitch_error = pid(current_pitch)  # PID calculates the control effort

    # Send the motor command to stabilize the head
    cmd.q = pitch_error  # Desired motor position (could be an angle adjustment)
    serial.sendRecv(cmd, data)  # Send the command to the motor and get feedback


    time.sleep(0.001)
    '''