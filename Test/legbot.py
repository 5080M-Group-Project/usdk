import time
import sys
import numpy as np
from scipy.linalg import solve_continuous_are
import os
import contextlib

# New IMU imports
import board
import adafruit_bno055

sys.path.append('../lib')
from unitree_actuator_sdk import *
from functions2 import *  # Includes getRotorGains()

# Open once at program start
_devnull = open(os.devnull, 'w')

@contextlib.contextmanager
def suppress_stdout_stderr():
    """Suppress C-level stdout and stderr."""
    old_stdout_fd = os.dup(1)
    old_stderr_fd = os.dup(2)

    os.dup2(_devnull.fileno(), 1)  # Redirect stdout
    os.dup2(_devnull.fileno(), 2)  # Redirect stderr

    try:
        yield
    finally:
        os.dup2(old_stdout_fd, 1)  # Restore stdout
        os.dup2(old_stderr_fd, 2)  # Restore stderr
        os.close(old_stdout_fd)
        os.close(old_stderr_fd)

# --- Setup Serial Communication ---
left = SerialPort('/dev/ttyUSB1')   # Left leg: hip(0), knee(1), wheel(2)
right = SerialPort('/dev/ttyUSB0')  # Right leg: hip(0), knee(1), wheel(2)

# --- Command and Data Structs ---
cmd = MotorCmd()
data = MotorData()
cmd.motorType = MotorType.A1
data.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)

# --- Gain tuning ---
kpOutWheel, kdOutWheel = 30, 5
kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)


gearRatio = queryGearRatio(MotorType.A1)

# Define hip and knee angles for both USB ports
# USB1
hip_angle_usb1 = 9.503  # rad
knee_angle_usb1 = -1.797  # rad

# USB0
hip_angle_usb0 = 1.672 # rad
knee_angle_usb0 = 10.259  # rad

# --- Hip and Knee Angles (rad) ---
#hip_angle_left = 9.503
#knee_angle_left = -1.797
#hip_angle_right = 1.672
#knee_angle_right = 10.259

# --- LQR system setup ---
wheel_radius = 0.05
body_mass = 8.79
robot_height = 0.175
g = 9.81
max_wheel_speed = 20  # rad/s

A = np.array([[0, 1, 0, 0],
              [0, 0, g / robot_height, 0],
              [0, 0, 0, 1],
              [0, 0, 0, 0]])

B = np.array([[0, 0],
              [1 / (body_mass * wheel_radius ** 2), 1 / (body_mass * wheel_radius ** 2)],
              [0, 0],
              [1 / (body_mass * wheel_radius), -1 / (body_mass * wheel_radius)]])

Q = np.diag([2000, 1, 0, 0]) # Penalties for pitch error, pitch rate, velocity error, and yaw rate error
R = np.diag([0.1, 0.1])
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P

base_pitch_offset = 0.06 # decreasing makes drift more backward, increasing causes foward drift
desired_velocity = 0.0
desired_yaw_rate = 0.0
wheel_separation = 0.2

v_left = 0
v_right = 0

# --- IMU Setup ---
i2c = board.I2C()
imu = adafruit_bno055.BNO055_I2C(i2c)

# --- Loop Settings ---
dt = 1 / 500
pitch = 0
try:
    while True:
        # --- Set hip and knee angles ---
        #for port, hip_angle, knee_angle in [
        #    (left, hip_angle_left, knee_angle_left),
        #    (right, hip_angle_right, knee_angle_right)
        #]:
        #    for motor_id, angle in zip([0, 1], [hip_angle, knee_angle]):
        #        cmd.id = motor_id
        #        cmd.kp = kpRotorWheel
        #        cmd.kd = kdRotorWheel
        #        cmd.q = angle
        #        port.sendRecv(cmd, data)
        # Send commands for USB1 (hip and knee motors)
        # Setup for USB1 - Hip and Knee Motors
        cmd.id = 0  # Hip motor ID for USB1
        cmd.dq = 0
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel
        cmd.q = hip_angle_usb1  # Command hip angle in radians
        with suppress_stdout_stderr():
            left.sendRecv(cmd, data)

        #print(f"USB1 - Hip Commanded Angle (rad): {hip_angle_usb1}")

        cmd.id = 1  # Knee motor ID for USB1
        cmd.dq = 0
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel
        cmd.q = knee_angle_usb1  # Command knee angle in radians
        with suppress_stdout_stderr():
            left.sendRecv(cmd, data)

        #print(f"USB1 - Knee Commanded Angle (rad): {knee_angle_usb1}")

        # Send commands for USB0 (hip and knee motors)
        cmd.id = 0  # Hip motor ID for USB0
        cmd.dq = 0
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel
        cmd.q = hip_angle_usb0  # Command hip angle in radians
        with suppress_stdout_stderr():
            right.sendRecv(cmd, data)

        #print(f"USB0 - Hip Commanded Angle (rad): {hip_angle_usb0}")

        cmd.id = 1  # Knee motor ID for USB0
        cmd.dq = 0
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel
        cmd.q = knee_angle_usb0  # Command knee angle in radians
        with suppress_stdout_stderr():
            right.sendRecv(cmd, data)

        #print(f"USB0 - Knee Commanded Angle (rad): {knee_angle_usb0}")
        # --- IMU readings ---
        euler = imu.euler
        gyro = imu.gyro

        # Pitch (euler[2])
        if euler[2] is not None:
            #if abs(euler[2])<100:
            if euler[2] < 0:
                pitch = -180 - euler[2]
            else:
                pitch = 180 - euler[2]
        else:
            pitch = pitch

        # Pitch rate (gyro[2])

        pitch_rate = -1 * gyro[2]


        # Read yawrate (not used yet)

        yawrate =  gyro[0]  # LEFT TURN IS POSITIVE


        # Skip loop if sensor failed
        if pitch is None or pitch_rate is None:
            time.sleep(dt)
            continue



        forward_velocity = 0#(v_left + v_right) / 2

        #forward_velocity = (v_left + v_right) / 2


        pitch_offset = base_pitch_offset #+ 0.04 * desired_velocity

        # --- LQR control ---
        state = np.array([
            np.radians(pitch),
            np.radians(pitch_rate),
            forward_velocity,
            yawrate
        ])
        control = -K @ (state - np.array([pitch_offset, 0, desired_velocity, desired_yaw_rate]))
        control = np.clip(control, -max_wheel_speed, max_wheel_speed)
        left_cmd, right_cmd = control

        # --- Send wheel commands (ID 2) ---
        for port, vel in [(left, -left_cmd), (right, right_cmd)]:
            cmd.id = 2
            cmd.kp = 0
            cmd.kd = 0.1

            cmd.dq = vel*9.1 #gear
            with suppress_stdout_stderr():
                while not port.sendRecv(cmd, data):
                    print("no data")
                if port == left:
                    v_left = (data.dq)/9.1
                else:
                    v_right = (data.dq)/9.1

            #forward_velocity = 0 #(v_left + v_right) / 2
        print(f"Pitch: {pitch:.2f}°, Rate: {pitch_rate:.2f}°/s, "
              f"Vel: {forward_velocity:.2f} m/s | L: {left_cmd:.2f}, R: {right_cmd:.2f}")

        # --- Wheel velocities ---

        time.sleep(dt)

except KeyboardInterrupt:
    print("\nLoop stopped by user.")
    sys.exit(0)
