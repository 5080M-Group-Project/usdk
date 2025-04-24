from unitree_motor import Controller
import time
import math

# Gear reduction (if output shaft angle is needed)
GEAR_RATIO = 9.0  # Adjust if different

# Create controllers for both legs
controller_leg1 = Controller('/dev/ttyUSB0', 115200)
controller_leg2 = Controller('/dev/ttyUSB1', 115200)

# Motor IDs per leg (common config: hip=1, knee=2, wheel=3)
motor_ids = [1, 2, 3]

def read_leg_angles(controller, leg_name):
    for motor_id in motor_ids:
        state = controller.read_motor_state(motor_id)
        rotor_angle_rad = state.q
        joint_angle_deg = math.degrees(rotor_angle_rad) / GEAR_RATIO
        print(f"{leg_name} - Motor {motor_id}: {joint_angle_deg:.2f}°")

print("Reading all 6 motor angles... Press Ctrl+C to stop.\n")

try:
    while True:
        read_leg_angles(controller_leg1, "Leg 1")
        read_leg_angles(controller_leg2, "Leg 2")
        print("-" * 50)
        time.sleep(1)

except KeyboardInterrupt:
    print("Stopped.")
