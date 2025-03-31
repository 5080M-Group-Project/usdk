import time
import sys
import numpy as np

sys.path.append('../lib')
from unitree_actuator_sdk import *

serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()

# Ensure a valid gear ratio
gear_ratio = queryGearRatio(MotorType.A1)
if gear_ratio <= 0:
    raise ValueError("Invalid gear ratio received. Check motor communication.")


def get_motor_angle(motor_id, retries=3):
    """ Reads the motor angle and ensures it's valid. Retries if needed. """
    cmd.motorType = MotorType.A1
    cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
    cmd.id = motor_id

    for _ in range(retries):
        serial.sendRecv(cmd, data)
        angle = (data.q / gear_ratio) * (180 / np.pi)

        # Check for absurdly large values (e.g., 10 times max rotation)
        if abs(data.q) < 10 * np.pi * gear_ratio:
            return angle

        print(f"Warning: Unreasonable encoder value for motor {motor_id}, retrying...")
        time.sleep(0.1)  # Wait before retrying

    raise ValueError(f"Motor {motor_id} failed to return a valid angle after {retries} attempts.")


# Get initial angles
try:
    hipInitial = get_motor_angle(0)
    kneeInitial = get_motor_angle(1)
except ValueError as e:
    print(e)
    sys.exit(1)

offsetHip = -90.0 - hipInitial
offsetKnee = 0.0 - kneeInitial
calibration = False

while True:
    if not calibration:
        try:
            hipInitial = get_motor_angle(0)
            kneeInitial = get_motor_angle(1)
        except ValueError as e:
            print(e)
            continue  # Retry on next loop iteration

        offsetHip = -90.0 - hipInitial
        offsetKnee = 0.0 - kneeInitial
        calibration = True
    else:
        # Read & display angles
        for motor_id, offset in [(0, offsetHip), (1, offsetKnee)]:
            try:
                angle = get_motor_angle(motor_id) + offset
                print(f"\nMotor {motor_id}:")
                print(f"Raw Output Angle (rad): {data.q / gear_ratio}")
                print(f"Raw Output Angle (deg): {(data.q / gear_ratio) * (180 / np.pi)}")
                print(f"Angle w. offset (deg): {angle}")
                print(f"Motor Error Code: {data.merror}\n")
            except ValueError as e:
                print(e)

        time.sleep(0.01)  # Allow time for serial communication
