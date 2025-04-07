import time
import math
import sys
import board
import busio
import adafruit_bno055

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
kpOutWheel, kdOutWheel = 7.0, 2.0
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
        current_motor_q += correction  # Increment motor position

        # Send updated position command
        cmd.q = current_motor_q
        cmd.dq = 1.0  # 1.0 #speed motor, maybe we can control this too
        cmd.tau = 0.0
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel
        success = serial.sendRecv(cmd, data)

        if success:
            print(
                f"Pitch: {pitch:.2f}°, Correction: {math.degrees(correction):+.2f}°, Motor q: {math.degrees(current_motor_q):.2f}°")
        else:
            print("❌ Motor communication error.")

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n🛑 Stopping stabilization.")
