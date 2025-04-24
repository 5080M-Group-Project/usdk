import time
import sys

sys.path.append('../lib')
from unitree_actuator_sdk import *
from functions2 import *

# --- Setup Serial Communication ---
left = SerialPort('/dev/ttyUSB1')
right = SerialPort('/dev/ttyUSB0')
# --- Initialize Loop Variables ---
hipTau, kneeTau = 0.0, 0.0  # Torque
timeSteps = []

# Gain tuning
kpOutWheel, kdOutWheel = 20, 5
kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)

# Define hip and knee angles for both USB ports
# USB1
hip_angle_usb1 = 6  # rad
knee_angle_usb1 = -3.82  # rad

# USB0
hip_angle_usb0 = 4.5 # rad
knee_angle_usb0 = 10.1687  # rad

# --- Initial motor setup ---
cmd = MotorCmd()
data = MotorData()
cmd.motorType = MotorType.A1
data.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)


# --- Main Loop ---
try:
    while True:
        # Send commands for USB1 (hip and knee motors)


        # Setup for USB1 - Hip and Knee Motors
        cmd.id = 0  # Hip motor ID for USB1
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel
        cmd.q = hip_angle_usb1  # Command hip angle in radians

        left.sendRecv(cmd, data)  # Send command to USB1 hip motor
        print(f"USB1 - Hip Commanded Angle (rad): {hip_angle_usb1}")

        cmd.id = 1  # Knee motor ID for USB1
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel
        cmd.q = knee_angle_usb1  # Command knee angle in radians
        left.sendRecv(cmd, data)  # Send command to USB1 knee motor
        print(f"USB1 - Knee Commanded Angle (rad): {knee_angle_usb1}")

        # Send commands for USB0 (hip and knee motors)
        cmd.id = 0  # Hip motor ID for USB0
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel
        cmd.q = hip_angle_usb0  # Command hip angle in radians
        right.sendRecv(cmd, data)  # Send command to USB0 hip motor
        print(f"USB0 - Hip Commanded Angle (rad): {hip_angle_usb0}")

        cmd.id = 1  # Knee motor ID for USB0
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel
        cmd.q = knee_angle_usb0  # Command knee angle in radians
        right.sendRecv(cmd, data)  # Send command to USB0 knee motor
        print(f"USB0 - Knee Commanded Angle (rad): {knee_angle_usb0}")

        # Wait a little before sending the next command
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nLoop stopped by user.")
    sys.exit(0)  # Ensure clean exit
