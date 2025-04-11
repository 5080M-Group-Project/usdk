import time
import math
import sys
import board
import busio
import adafruit_bno055
from datetime import datetime
from simple_pid import PID

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
kpOutWheel, kdOutWheel = 2.0, 0.50
kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)

# --- Setup IMU ---
i2c = board.I2C()
imu = adafruit_bno055.BNO055_I2C(i2c)

# --- Setup PID ---
#1.5,0,0.5
pid = PID(.5, 0.0, 0.5, setpoint=0.0)  # Tune these later
pid.sample_time = 0.01
pid.output_limits = (-math.radians(20), math.radians(20))  # motor command in radians

# --- Initial motor setup ---
cmd.q = 0.0
cmd.dq = 0.0
cmd.tau = 0.0
cmd.kp = 0.0
cmd.kd = 0.0
serial.sendRecv(cmd, data)
time.sleep(0.1)

# Logging setup
log_filename = f"pitch_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
log_file = open(log_filename, "w", newline="")
csv_writer = csv.writer(log_file)
csv_writer.writerow(["Time (s)", "Pitch (deg)", "PID Correction (deg)", "Motor Position q (deg)"])


print("🟢 Pitch stabilization running...")

try:
    current_motor_q = 0.0  # We accumulate changes to q here

    while True:
        euler = imu.euler
        pitch = euler[1] if euler else None  # Pitch in degrees

        if pitch is None:
            print("⚠️ IMU is blid or mute")
            time.sleep(0.01)
            continue

        correction = pid(pitch)  # PID returns delta q in radians

        # Apply correction to target position
        current_motor_q += correction  # This accumulates q over time

        # Send motor command
        cmd.q = current_motor_q
        cmd.dq = 0.0 #1.0 #speed motor, maybe we can control this too
        cmd.tau = 0.0
        cmd.kp = kpRotorWheel
        cmd.kd = kdRotorWheel

        success = serial.sendRecv(cmd, data)
        if success:
            elapsed_time = time.time() - start_time
            print(f"[{elapsed_time:.2f}s] Pitch: {pitch:.2f}°, Correction: {math.degrees(correction):+.2f}°, Motor q: {math.degrees(current_motor_q):.2f}°")

            #print(f"Pitch: {pitch:.2f}°, Correction: {math.degrees(correction):+.2f}°, Motor q: {math.degrees(current_motor_q):.2f}°")
        # Log data
            csv_writer.writerow([
                elapsed_time,
                pitch,
                math.degrees(correction),
                math.degrees(current_motor_q)
            ])
        else:
            print("❌ Motor pooooooooo.")

        #time.sleep(0.01)

except KeyboardInterrupt:
    print("\n🛑 Watch behind you")
    log_file.close()
