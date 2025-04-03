import evdev


def find_f710():
    """Find the Logitech F710 controller device."""
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for device in devices:
        if "Logitech Gamepad F710" in device.name:
            print(f"Found Logitech F710 at {device.path}")
            return evdev.InputDevice(device.path)
    print("Logitech F710 not found. Ensure it is connected and in DirectInput mode.")
    exit()


# Connect to the F710 device
gamepad = find_f710()

print("Listening for controller input. Move the sticks or press buttons...")

# Mapping of joystick axes (modify if needed)
axis_mapping = {
    0: "Left Stick X",  # Left-right movement on left stick
    1: "Left Stick Y",  # Up-down movement on left stick
    2: "Right Stick X",  # Left-right movement on right stick
    3: "Right Stick Y",  # Up-down movement on right stick
}

# Read events from the controller
for event in gamepad.read_loop():
    if event.type == evdev.ecodes.EV_ABS:  # Joystick movement
        axis = evdev.categorize(event)
        if event.code in axis_mapping:
            print(f"{axis_mapping[event.code]}: {event.value}")

    elif event.type == evdev.ecodes.EV_KEY:  # Button press
        if event.value == 1:  # Button pressed
            print(f"Button {event.code} pressed")
        elif event.value == 0:  # Button released
            print(f"Button {event.code} released")
