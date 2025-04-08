import time
import math
import sys
import board
import busio
import adafruit_bno055
import pygame

# Initialize pygame
pygame.init()

# Initialize joystick
pygame.joystick.init()

# Ensure at least one joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick detected!")
    exit()

# Get the first joystick (F710 should be js0)
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Connected to: {joystick.get_name()}")

# Tracking button states
prev_axis1 = -0.00
prev_axis0 = -0.00

sys.path.append('../lib')
from unitree_actuator_sdk import *
from functionsIMU import *  # Should include getOutputAngleDeg()

# --- Setup motor ---
serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()
cmd.id = 0
cmd.motorType = MotorType.A1
data.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)

# Gain tuning
kpOutWheel, kdOutWheel = 15.0, 1 #22.4, 1.5
kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)


# --- Setup IMU ---
i2c = board.I2C()
imu = adafruit_bno055.BNO055_I2C(i2c)

# --- Initial motor setup ---
cmd.q = 0.0
cmd.dq = 0.0
cmd.tau = 0.0
cmd.kp = 0.0
cmd.kd = 0.0
serial.sendRecv(cmd, data)
time.sleep(0.1)

print("🟢 Pitch stabilization running...")

try:
    current_motor_q = 0.0  # Track motor position

    while True:
        euler = imu.euler
        pitch = euler[1] if euler else None  # Pitch in degrees

        if pitch is None:
            print("⚠️ IMU is not responding")
            time.sleep(0.01)
            continue

        # Compute relative correction
        correction = -math.radians(pitch)  # Convert degrees to radians
        if correction > math.radians(20):
            correction = math.radians(20)
        elif correction < math.radians(-20):
            correction = math.radians(-20)

        #current_motor_q += correction  # Increment motor position
        pygame.event.pump()  # Process events

        # Read joystick inputs
        axis_0 = round(joystick.get_axis(0), 2)  # Left stick X-axis
        axis_1 = round(joystick.get_axis(1), 2)  # Left stick Y-axis


        # Detect single press event for button0 (increment kd)
        if axis_1 == -1.00 and prev_axis1 == -0.00:
            kdOutWheel += 0.1
            print(f"dpad up pressed, kd increased to {kdOutWheel:.1f}")

        # Detect single press event for button1 (decrement kd)
        if axis_1 == 1.00 and prev_axis1 == -0.00:
            kdOutWheel -= 0.1
            print(f"dpad down pressed, kd decreased to {kdOutWheel:.1f}")

        if axis_0 == -1.00 and prev_axis0 == -0.00:
            kpOutWheel -= 0.1
            print(f"dpad up pressed, kd increased to {kpOutWheel:.1f}")

            # Detect single press event for button1 (decrement kd)
        if axis_0 == 1.00 and prev_axis0 == -0.00:
            kpOutWheel += 0.1
            print(f"dpad down pressed, kd decreased to {kpOutWheel:.1f}")
        # Update previous button states
        prev_axis1 = axis_1
        prev_axis0 = axis_0
        # prev_button1 = button1

        # Print joystick values
        print(
            f"Left Stick: ({axis_0:.2f}, {axis_1:.2f}) | kd: {kdOutWheel:.1f}, kp: {kpOutWheel:.1f}")

        # Send updated position command
        current_motor_q += correction  # Increment motor position
        cmd.q = current_motor_q
        #cmd.dq = 5  # 1.0 #speed motor, maybe we can control this too
        cmd.tau = 0.0
        kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel
        success = serial.sendRecv(cmd, data)

       # if success:
            #print(
               # f"Pitch: {pitch:.2f}°, Correction: {math.degrees(correction):+.2f}°, Motor q: {math.degrees(current_motor_q):.2f}°")
        #else:
             #print("❌ Motor communication error.")

        #time.sleep(0.01)

except KeyboardInterrupt:
    print("\n🛑 Stopping stabilization.")
