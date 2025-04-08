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
prev_axis1 = 0
#prev_button1 = 0
kd = 1.0  # Example starting value

while True:
    pygame.event.pump()  # Process events

    # Read joystick inputs
    axis_0 = joystick.get_axis(0)  # Left stick X-axis
    axis_1 = joystick.get_axis(1)  # Left stick Y-axis
    axis_2 = joystick.get_axis(2)  # Right stick X-axis
    axis_3 = joystick.get_axis(3)  # Right stick Y-axis
    button0 = joystick.get_button(0)
    button1 = joystick.get_button(1)

    # Detect single press event for button0 (increment kd)
    if axis_1 == -1 and prev_axis1 == -0.00:
        kd += 0.1
        print(f"dpad up pressed, kd increased to {kd:.1f}")

    # Detect single press event for button1 (decrement kd)
    if axis_1 == 1 and prev_axis1 == -0.00:
        kd -= 0.1
        print(f"dpad down pressed, kd decreased to {kd:.1f}")

    # Update previous button states
    prev_axis1 = axis_1
    #prev_button1 = button1

    # Print joystick values
    print(
        f"Left Stick: ({axis_0:.2f}, {axis_1:.2f}) | Right Stick: ({axis_2:.2f}, {axis_3:.2f}) | Button0: {button0}, Button1: {button1} | kd: {kd}")

