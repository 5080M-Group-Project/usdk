from functions import *
import time
import pygame
import sys

sys.path.append('../lib')
from unitree_actuator_sdk import *

# Initialize Serial Ports
serial_1 = SerialPort('/dev/ttyUSB0')  # Leg 1
serial_2 = SerialPort('/dev/ttyUSB1')  # Leg 2

# Data logging
timeSteps = []

# Offsets, torque init
hipOffset_1, kneeOffset_1 = 0.0, 0.0
hipOffset_2, kneeOffset_2 = 0.0, 0.0
hipTau, kneeTau = 0.0, 0.0

offsetCalibration_1 = False
offsetCalibration_2 = False

# Crouch init
crouching = False
crouchHeightDesiredPrev = crouchHeightMax
crouchHeightDesiredNew = 0.75 * crouchHeightMax
crouchDuration = 0.625
crouchIncrement = 0.15 * crouchHeightMax

# Calibration loop
while not (offsetCalibration_1 and offsetCalibration_2):
    if not offsetCalibration_1:
        hipOffset_1, kneeOffset_1, hipOutputAngleDesired_1, kneeOutputAngleDesired_1, offsetCalibration_1 = calibrateJointReadings(serial_1)
    if not offsetCalibration_2:
        hipOffset_2, kneeOffset_2, hipOutputAngleDesired_2, kneeOutputAngleDesired_2, offsetCalibration_2 = calibrateJointReadings(serial_2)
    time.sleep(0.1)

hipOutputAngleCurrent_1, kneeOutputAngleCurrent_1 = hipOutputAngleDesired_1, kneeOutputAngleDesired_1
hipOutputAngleCurrent_2, kneeOutputAngleCurrent_2 = hipOutputAngleDesired_2, kneeOutputAngleDesired_2

globalStartTime = time.time()

try:
    while True:
        loopStartTime = time.time()
        elapsedTime = loopStartTime - globalStartTime
        timeSteps.append(elapsedTime)

        events = pygame.event.get()
        crouchHeightDesiredNew = getCrouchCommand(events, crouchHeightDesiredNew, crouchIncrement)

        for serial, id_suffix, hipOffset, kneeOffset, hipOutputAngleDesired, kneeOutputAngleDesired, hipOutputAngleCurrent, kneeOutputAngleCurrent in [
            (serial_1, "_1", hipOffset_1, kneeOffset_1, hipOutputAngleDesired_1, kneeOutputAngleDesired_1, hipOutputAngleCurrent_1, kneeOutputAngleCurrent_1),
            (serial_2, "_2", hipOffset_2, kneeOffset_2, hipOutputAngleDesired_2, kneeOutputAngleDesired_2, hipOutputAngleCurrent_2, kneeOutputAngleCurrent_2),
        ]:

            hipRotorAngleDesired = getRotorAngleRad(hipOutputAngleDesired - hipOffset)
            kneeRotorAngleDesired = getRotorAngleRad(kneeOutputAngleDesired - kneeOffset)
            kpRotorHip, kdRotorHip, kpRotorKnee, kdRotorKnee = chooseRotorGains(crouching)

            # Hip
            data = sendCmdRcvData(serial, id.hip, kpRotorHip, kdRotorHip, hipRotorAngleDesired, 0.0, hipTau)
            hipOutputAngleCurrent = getOutputAngleDeg(data.q) + hipOffset

            # Knee
            data = sendCmdRcvData(serial, id.knee, kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired, 0.0, kneeTau)
            kneeOutputAngleCurrent = getOutputAngleDeg(data.q) + kneeOffset

            # Crouch update
            hipOutputAngleDesired, kneeOutputAngleDesired, crouchHeightDesiredPrev, crouching = crouchControl(
                'front', hipOutputAngleCurrent, kneeOutputAngleCurrent,
                crouchHeightDesiredPrev, crouchHeightDesiredNew,
                crouchDuration, crouching
            )

            # Store back into appropriate vars
            if id_suffix == "_1":
                hipOutputAngleDesired_1, kneeOutputAngleDesired_1 = hipOutputAngleDesired, kneeOutputAngleDesired
                hipOutputAngleCurrent_1, kneeOutputAngleCurrent_1 = hipOutputAngleCurrent, kneeOutputAngleCurrent
            else:
                hipOutputAngleDesired_2, kneeOutputAngleDesired_2 = hipOutputAngleDesired, kneeOutputAngleDesired
                hipOutputAngleCurrent_2, kneeOutputAngleCurrent_2 = hipOutputAngleCurrent, kneeOutputAngleCurrent

        loopTime = time.time() - loopStartTime
        print(f"Loop Time: {loopTime:.4f}s\n")

except KeyboardInterrupt:
    print("Exiting cleanly.")
    sys.exit(0)
