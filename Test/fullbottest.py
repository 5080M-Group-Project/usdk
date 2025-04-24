import time
import sys
from unitree_actuator_sdk import *

# --- Setup Serial Communication ---
left = SerialPort('/dev/ttyUSB1')

# --- Initialize Loop Variables ---
hipTau, kneeTau = 0.0, 0.0  # Torque
timeSteps = []

# Define hip and knee angles for both USB ports
# USB1
hip_angle_usb1 = 9.0456  # rad
knee_angle_usb1 = -10.1485  # rad

# USB0
hip_angle_usb0 = 2.0458  # rad
knee_angle_usb0 = 10.1687  # rad

# --- Initial motor setup ---
cmd.q = 0.0
cmd.dq = 0.0
cmd.tau = 0.0
cmd.kp = 0.0
cmd.kd = 0.0
left.sendRecv(cmd, data)
time.sleep(0.1)

# --- Main Loop ---
try:
    while True:
        # Send commands for USB1 (hip and knee motors)
        cmd = MotorCmd()
        data = MotorData()

        # Setup for USB1 - Hip and Knee Motors
        cmd.id = 0  # Hip motor ID for USB1
        cmd.motorType = MotorType.A1
        cmd.mode = MotorMode.POSITION_CONTROL
        cmd.q = hip_angle_usb1  # Command hip angle in radians
        cmd.dq = 0.0  # No speed control
        cmd.tau = hipTau  # Torque
        left.sendRecv(cmd, data)  # Send command to USB1 hip motor
        print(f"USB1 - Hip Commanded Angle (rad): {hip_angle_usb1}")

        cmd.id = 1  # Knee motor ID for USB1
        cmd.q = knee_angle_usb1  # Command knee angle in radians
        left.sendRecv(cmd, data)  # Send command to USB1 knee motor
        print(f"USB1 - Knee Commanded Angle (rad): {knee_angle_usb1}")

        # Send commands for USB0 (hip and knee motors)
        cmd.id = 0  # Hip motor ID for USB0
        cmd.q = hip_angle_usb0  # Command hip angle in radians
        left.sendRecv(cmd, data)  # Send command to USB0 hip motor
        print(f"USB0 - Hip Commanded Angle (rad): {hip_angle_usb0}")

        cmd.id = 1  # Knee motor ID for USB0
        cmd.q = knee_angle_usb0  # Command knee angle in radians
        left.sendRecv(cmd, data)  # Send command to USB0 knee motor
        print(f"USB0 - Knee Commanded Angle (rad): {knee_angle_usb0}")

        # Wait a little before sending the next command
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nLoop stopped by user.")
    sys.exit(0)  # Ensure clean exit
