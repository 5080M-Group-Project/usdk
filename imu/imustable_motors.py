import time
import math
import serial
import numpy as np
from simple_pid import PID
from adafruit_bno055 import BNO055
import board
import busio

# Initialize I2C for the BNO055 IMU
i2c = board.I2C()
imu = adafruit_bno055.BNO055_I2C(i2c)


# Initialize serial communication with the motor
serial_port = '/dev/ttyUSB0'  # Adjust as needed
baud_rate = 115200
ser = serial.Serial(serial_port, baud_rate, timeout=1)
cmd = MotorCmd()
data = MotorData()
# PID Controller for position control
kp, ki, kd = 3.0, 0.1, 0.5  # Tune these gains as needed
pid = PID(kp, ki, kd, setpoint=0)
pid.output_limits = (-math.radians(10), math.radians(10))  # Limit output to ±10 degrees

def get_pitch():
    """Returns the current pitch angle from the IMU in degrees."""
    pitch = imu.euler[1]  # BNO055 euler[1] gives pitch
    return pitch if pitch is not None else 0.0

def send_motor_command(position):
    """Sends position command to the motor."""
    # Construct the command packet according to Unitree's communication protocol
    # This is a placeholder; refer to Unitree's SDK documentation for exact packet structure
    command = f"#{position}\n"
    cmd.q = position
    serial.sendRecv(cmd, data)

def balance_motor():
    """Continuously adjusts motor position to maintain zero pitch."""
    while True:
        # Read current pitch angle
        pitch = get_pitch()

        # Compute PID control output (desired position adjustment)
        position_adjustment = pid(pitch)

        # Send position command to motor
        send_motor_command(position_adjustment)
        mangle = str(data.q)
        # Debugging output
        print(f"Pitch: {pitch:.2f} deg, Position Adjustment: {math.degrees(position_adjustment):.2f} deg")
        print(f"Motor angle: {mangle:.2f} deg,")
        time.sleep(0.01)  # 10ms loop time

if __name__ == "__main__":
    try:
        balance_motor()
    except KeyboardInterrupt:
        print("\nStopping motor...")
        send_motor_command(0)  # Send neutral command to stop motor safely
