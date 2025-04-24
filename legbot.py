import time
import math
import sys
import board
import busio
import adafruit_bno055
import numpy as np
from scipy.linalg import solve_continuous_are

# === UNITREE SDK ===
sys.path.append('../lib')
from unitree_actuator_sdk import *

# === IMU SETUP ===
i2c = board.I2C()
imu = adafruit_bno055.BNO055_I2C(i2c)

# === MOTOR SETUP ===
serial = SerialPort('/dev/ttyUSB0')

# Joint IDs (update if needed)
LEFT_WHEEL = 1
RIGHT_WHEEL = 2
HIP_JOINT = 3
KNEE_JOINT = 4

# Create command and data objects
motors = {}
for motor_id in [LEFT_WHEEL, RIGHT_WHEEL, HIP_JOINT, KNEE_JOINT]:
    cmd = MotorCmd()
    data = MotorData()
    cmd.id = motor_id
    cmd.motorType = MotorType.A1
    data.motorType = MotorType.A1
    cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
    cmd.q = 0.0
    cmd.dq = 0.0
    cmd.tau = 0.0
    cmd.kp = 0.0
    cmd.kd = 0.0
    motors[motor_id] = {'cmd': cmd, 'data': data}

# === ROBOT PARAMETERS ===
wheel_radius = 0.05  # meters
body_mass = 6.37  # kg
robot_height = 0.20  # meters
g = 9.81
max_wheel_speed = 20.0  # rad/s
base_pitch_offset = 0.07  # radians

# === LQR SETUP ===
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

# === CONTROL TARGETS ===
desired_velocity = 0.0  # m/s
desired_yaw_rate = 0.0  # rad/s
desired_hip_angle = 0.0  # rad
desired_knee_angle = 0.0  # rad

# === FUNCTION TO SEND POSITION COMMAND ===
def send_position_command(joint_id, target_angle):
    cmd = motors[joint_id]['cmd']
    cmd.q = target_angle
    cmd.dq = 0.0
    cmd.tau = 0.0
    cmd.kp = 25.0
    cmd.kd = 1.0
    serial.sendRecv(cmd, motors[joint_id]['data'])

# === FUNCTION TO SEND WHEEL SPEED COMMAND ===
def send_wheel_speed(joint_id, speed):
    cmd = motors[joint_id]['cmd']
    cmd.q = 0.0
    cmd.dq = speed
    cmd.tau = 0.0
    cmd.kp = 0.0
    cmd.kd = 0.5
    serial.sendRecv(cmd, motors[joint_id]['data'])

# === LQR CONTROL ===
def lqr_control(state, velocity_ref, yaw_ref, pitch_offset):
    error = np.array([
        state[0] - pitch_offset,
        state[1],
        state[2] - velocity_ref,
        state[3] - yaw_ref
    ])
    control = -K @ error
    return np.clip(control, -max_wheel_speed, max_wheel_speed)

# === MAIN LOOP ===
print("🟢 LQR Balancing Started")
loop_dt = 1 / 240
last_pitch = 0.0
last_time = time.time()

try:
    while True:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        euler = imu.euler
        pitch_deg = euler[1] if euler else None
        if pitch_deg is None:
            print("⚠️ IMU not responding")
            time.sleep(0.01)
            continue

        pitch = math.radians(pitch_deg)
        pitch_rate = imu.gyro[1] if imu.gyro else 0.0
        yaw_rate = imu.gyro[2] if imu.gyro else 0.0

        forward_velocity = (pitch - last_pitch) / dt
        last_pitch = pitch

        # Adjust pitch offset dynamically
        pitch_offset = base_pitch_offset + 0.04 * desired_velocity

        # LQR controller
        state = np.array([pitch, pitch_rate, forward_velocity, yaw_rate])
        left_speed, right_speed = lqr_control(state, desired_velocity, desired_yaw_rate, pitch_offset)

        # Send commands to wheels
        send_wheel_speed(LEFT_WHEEL, -left_speed)
        send_wheel_speed(RIGHT_WHEEL, -right_speed)

        # Send static joint positions
        send_position_command(HIP_JOINT, desired_hip_angle)
        send_position_command(KNEE_JOINT, desired_knee_angle)

        print(f"Pitch: {pitch_deg:+.2f}°, Vel: {forward_velocity:.2f}, L: {left_speed:.2f}, R: {right_speed:.2f}")
        time.sleep(loop_dt)

except KeyboardInterrupt:
    print("🛑 Balancing stopped.")
