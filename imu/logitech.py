import evdev

# Manually set the device path (change if needed)
#DEVICE_PATH = "/dev/input/eventX"  # Replace with the correct event number
# OR try: DEVICE_PATH = "/dev/input/js0"
DEVICE_PATH = "/dev/input/js0"
try:
    gamepad = evdev.InputDevice(DEVICE_PATH)
    print(f"Connected to {gamepad.name} at {DEVICE_PATH}")
except FileNotFoundError:
    print(f"Device not found at {DEVICE_PATH}. Run `ls /dev/input/` to check the available inputs.")
    exit()

print("Listening for controller input...")

for event in gamepad.read_loop():
    if event.type == evdev.ecodes.EV_ABS:  # Joystick movement
        print(f"Axis {event.code}: {event.value}")
    elif event.type == evdev.ecodes.EV_KEY:  # Button press
        print(f"Button {event.code} {'pressed' if event.value else 'released'}")
