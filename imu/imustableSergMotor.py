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
cmd.id = 1

motor_angle_initial_deg = getOutputAngleDeg(data.q)  # Get current motor angle in degrees


while True:
    # Get current pitch angle from the IMU
    current_pitch = imu.euler[1]  # Assuming this function returns the pitch in degrees

    # Calculate the error (difference between desired and current pitch)
    pitch_error = pid(current_pitch)  # PID calculates the control effort

    # Send the motor command to stabilize the head
    cmd.q = pitch_error  # Desired motor position (could be an angle adjustment)
    serial.sendRecv(cmd, data)  # Send the command to the motor and get feedback


    time.sleep(0.001)