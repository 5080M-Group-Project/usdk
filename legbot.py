import time
import sys
import numpy as np
from scipy.linalg import solve_continuous_are

sys.path.append('../lib')
from unitree_actuator_sdk import *
from functions2 import *  # Includes getRotorGains()
from bno055 import BNO055  # Real IMU

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
kpOutWheel, kdOutWheel = 20, 5
kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)

# --- Hip and Knee Angles (rad) ---
hip_angle_left = 9.503
knee_angle_left = -1.797
hip_angle_right = 1.672
knee_angle_right = 10.259

# --- LQR system setup ---
wheel_radius = 0.05
body_mass = 6.37
robot_height = 0.20
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

Q = np.diag([100, 1, 180, 10])
R = np.diag([0.1, 0.1])
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P

base_pitch_offset = 0.07
desired_velocity = 0.0
desired_yaw_rate = 0.0
wheel_separation = 0.2

# --- IMU Setup ---
imu = BNO055(serial_port='/dev/ttyUSB2')
if not imu.begin():
    raise RuntimeError("Failed to initialize BNO055")

# --- Loop Settings ---
dt = 1 / 240

try:
    while True:
        # --- Set hip and knee angles ---
        for port, hip_angle, knee_angle in [
            (left, hip_angle_left, knee_angle_left),
            (right, hip_angle_right, knee_angle_right)
        ]:
            for motor_id, angle in zip([0, 1], [hip_angle, knee_angle]):
                cmd.id = motor_id
                cmd.kp = kpRotorWheel
                cmd.kd = kdRotorWheel
                cmd.q = angle
                port.sendRecv(cmd, data)

        # --- IMU readings ---
        pitch, _, _ = imu.read_euler()
        pitch_rate = imu.read_gyro()[1]

        # --- Wheel velocities ---
        for port, side in [(left, 'left'), (right, 'right')]:
            cmd.id = 2
            port.sendRecv(cmd, data)
            if side == 'left':
                v_left = data.dq * wheel_radius
            else:
                v_right = data.dq * wheel_radius

        forward_velocity = (v_left + v_right) / 2
        yaw_rate = (v_right - v_left) / wheel_separation
        pitch_offset = base_pitch_offset + 0.04 * desired_velocity

        # --- LQR control ---
        state = np.array([
            np.radians(pitch),
            np.radians(pitch_rate),
            forward_velocity,
            yaw_rate
        ])
        control = -K @ (state - np.array([pitch_offset, 0, desired_velocity, desired_yaw_rate]))
        control = np.clip(control, -max_wheel_speed, max_wheel_speed)
        left_cmd, right_cmd = control

        # --- Send wheel commands (ID 2) ---
        for port, vel in [(left, -left_cmd), (right, -right_cmd)]:
            cmd.id = 2
            cmd.kp = 0
            cmd.kd = 0.3
            cmd.q = 0
            cmd.dq = vel
            cmd.tau = 0
            port.sendRecv(cmd, data)

        print(f"Pitch: {pitch:.2f}°, Rate: {pitch_rate:.2f}°/s, "
              f"Vel: {forward_velocity:.2f} m/s | L: {left_cmd:.2f}, R: {right_cmd:.2f}")

        time.sleep(dt)

except KeyboardInterrupt:
    print("\nLoop stopped by user.")
    sys.exit(0)
