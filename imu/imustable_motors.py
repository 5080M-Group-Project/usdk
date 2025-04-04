import time
import math
import numpy as np
import serial
from simple_pid import PID
import board
import busio
from adafruit_bno055 import BNO055

from unitree_actuator_sdk import *
from functions import getRotorGains, getRotorAngleRad, getOutputAngleDeg, calculateOutputTorque, cmdActuator, outputData

### === Init IMU === ###
i2c = busio.I2C(board.SCL, board.SDA)
imu = BNO055(i2c)

### === Init Motor === ###
serial = SerialPort('/dev/ttyUSB0')  # Make sure ttyUSB0 is correct
cmd = MotorCmd()
data = MotorData()

cmd.motorType = MotorType.A1
data.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)

motor_id = 0  # Replace with actual motor ID, e.g., id.hip

### === PID Controller === ###
kp, ki, kd = 3.0, 0.0, 0.3
pid = PID(kp, ki, kd, setpoint=0.0)  # Target pitch = 0 degrees
pid.output_limits = (-10, 10)  # Output in degrees

### === Gains for Motor Position Control === ###
kpOut, kdOut = 10.0, 0.2
kpRotor, kdRotor = getRotorGains(kpOut, kdOut)

### === Helper === ###
def get_pitch():
    pitch = imu.euler[1]
    return pitch if pitch is not None else 0.0

### === Balance Loop === ###
def balance_motor():
    while True:
        pitch = get_pitch()  # In degrees
        desired_angle_deg = pid(pitch)  # PID output in degrees

        desired_angle_rad = getRotorAngleRad(desired_angle_deg)  # Convert to rotor space (radians)

        # Send command
        cmd.id = motor_id
        cmd.kp = kpRotor
        cmd.kd = kdRotor
        cmd.q = desired_angle_rad
        cmd.dq = 0.0
        cmd.tau = 0.0

        if serial.sendRecv(cmd, data):
            actual_output_angle = getOutputAngleDeg(data.q)
        else:
            print("[WARNING] No motor response.")
            continue

        print(f"Pitch: {pitch:.2f}°, Command: {desired_angle_deg:.2f}°, Actual: {actual_output_angle:.2f}°")
        time.sleep(0.01)

### === Main === ###
if __name__ == "__main__":
    try:
        balance_motor()
    except KeyboardInterrupt:
        print("Stopping...")
        cmdActuator(motor_id, 0, 0, 0, 0, 0)
