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
kpOutWheel, kdOutWheel = 0, 1
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




        # Send updated position command
        cmd.motorType = MotorType.A1
        data.motorType = MotorType.A1
        cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
        speed = 3*pitch
        cmd.dq = speed
        cmd.tau = 0.0
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel
        success = serial.sendRecv(cmd, data)

        if success:
            print(
                f"Pitch: {pitch:.2f}°, dq: {speed:+.2f}")
        else:
            print("❌ Motor communication error.")

        #time.sleep(0.01)

except KeyboardInterrupt:
    print("\n🛑 Stopping stabilization.")
