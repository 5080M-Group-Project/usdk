import time
import sys
import board
import adafruit_bno055
import os
import contextlib

sys.path.append('../lib')
from unitree_actuator_sdk import *
from functions2 import *

@contextlib.contextmanager
def suppress_stdout_stderr():
    """Suppress C-level stdout and stderr (e.g., from C libraries like Unitree SDK)."""
    with open(os.devnull, 'w') as devnull:
        old_stdout_fd = os.dup(1)
        old_stderr_fd = os.dup(2)

        os.dup2(devnull.fileno(), 1)  # Redirect stdout (fd 1)
        os.dup2(devnull.fileno(), 2)  # Redirect stderr (fd 2)

        try:
            yield
        finally:
            os.dup2(old_stdout_fd, 1)  # Restore stdout
            os.dup2(old_stderr_fd, 2)  # Restore stderr

# --- Setup Serial Communication ---
left = SerialPort('/dev/ttyUSB1')
right = SerialPort('/dev/ttyUSB0')
# --- Initialize Loop Variables ---

timeSteps = []

# Gain tuning
kpOutWheel, kdOutWheel = 20, 5
kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)

# Define hip and knee angles for both USB ports
# USB1
hip_angle_usb1 = 9.503  # rad
knee_angle_usb1 = -1.797  # rad

# USB0
hip_angle_usb0 = 1.672 # rad
knee_angle_usb0 = 10.259  # rad

# --- Initial motor setup ---
cmd = MotorCmd()
data = MotorData()
cmd.motorType = MotorType.A1
data.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)

# --- IMU Setup ---
i2c = board.I2C()
imu = adafruit_bno055.BNO055_I2C(i2c)
pitch = 0
# --- Main Loop ---
try:
    while True:
        # Send commands for USB1 (hip and knee motors)
        # Setup for USB1 - Hip and Knee Motors
        cmd.id = 0  # Hip motor ID for USB1
        cmd.dq = 0
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel
        cmd.q = hip_angle_usb1  # Command hip angle in radians
        with suppress_stdout_stderr():
            left.sendRecv(cmd, data)

        angle = data.q
        #print(f"Raw left hip angle reading (radx9):  {angle} ")
        #print(f"USB1 - Hip Commanded Angle (rad): {hip_angle_usb1}")

        cmd.id = 1  # Knee motor ID for USB1
        cmd.dq = 0
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel
        cmd.q = knee_angle_usb1  # Command knee angle in radians
        with suppress_stdout_stderr():
            left.sendRecv(cmd, data)

        angle = data.q
        #print(f"Raw left knee angle reading (radx9):  {angle} ")
        #print(f"USB1 - Knee Commanded Angle (rad): {knee_angle_usb1}")

        # Send commands for USB0 (hip and knee motors)
        cmd.id = 0  # Hip motor ID for USB0
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel
        cmd.q = hip_angle_usb0  # Command hip angle in radians
        with suppress_stdout_stderr():
            right.sendRecv(cmd, data)

        angle = data.q
        #print(f"Raw right hip angle reading (radx9):  {angle} ")
        #print(f"USB0 - Hip Commanded Angle (rad): {hip_angle_usb0}")

        cmd.id = 1  # Knee motor ID for USB0
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel
        cmd.q = knee_angle_usb0  # Command knee angle in radians
        with suppress_stdout_stderr():
            right.sendRecv(cmd, data)

        angle = data.q
        #print(f"Raw right knee angle reading (radx9):  {angle} ")
        #print(f"USB0 - Knee Commanded Angle (rad): {knee_angle_usb0}")

        #CONTROL LOOP

        cmd.id = 2  # Knee motor ID for USB0
        cmd.kp = 0.0
        cmd.kd = 1.0*100/81
        cmd.dq = -54.0  # Command knee angle in radians
        with suppress_stdout_stderr():
            right.sendRecv(cmd, data)

        angle = data.q
        
        cmd.id = 2  # Knee motor ID for USB1
        cmd.kp = 0.0
        cmd.kd = 0.9
        cmd.dq = 54.0 # Command knee angle in radians
        with suppress_stdout_stderr():
            left.sendRecv(cmd, data)

        angle = data.q

        euler = imu.euler
        if euler[2] is not None:
            if euler[2] < 0:
                pitch = -180 - euler[2]
            else:
                pitch = 180 - euler[2]
        else:
            pitch = pitch
        print(f"Pitch:  {pitch} ")

        # Wait a little before sending the next command
        time.sleep(0)

except KeyboardInterrupt:
    print("\nLoop stopped by user.")
    sys.exit(0)  # Ensure clean exit
