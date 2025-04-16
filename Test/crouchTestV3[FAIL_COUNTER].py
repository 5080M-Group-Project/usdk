import time
import sys
from typing import Any
import numpy as np
import matplotlib.pyplot as plt
import pygame

from functions import *

######## NEEDED??? ########
sys.path.append('../lib')
from unitree_actuator_sdk import *

serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()

cmd.motorType = MotorType.A1
data.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)

pygame.init()
screen = pygame.display.set_mode((400, 300)) # NEEDED????
############################

##### NOTE 1: All rotor angles in RAD, all output angles in DEG########
##### NOTE 2: Whenever reading angles +offset, whenever commanding -offset. Offset in DEG######

# Initialize Motor Gains - add to .h file or equivalent

#Initialize Loop Variables

# Data storage for plotting
hipOutputAngles, hipCommandAngles, hipOutputTorque, kneeOutputAngles, kneeCommandAngles, kneeOutputTorque, timeSteps = [], [], [], [], [], [], []


#NEEDED?
hipOffset, kneeOffset = 0.0, 0.0
hipOutputAngleDesired, kneeOutputAngleDesired = 0.0, 0.0


hipTau, kneeTau = 0.0, 0.0


offsetCalibration = False
#positionCalibration = False ????

#Crouching Initialisation
crouching = False
crouchHeightDesiredPrev = crouchHeightMax
crouchHeightDesiredNew = 0.6*crouchHeightMax
crouchDuration = 1.0 #### scale by the distance required?
crouchIncrement = 0.25*crouchHeightMax



try:
        while True:
                while not offsetCalibration: ### & other
                        hipOffset, kneeOffset, hipOutputAngleDesired, kneeOutputAngleDesired, offsetCalibration = calibrateJointReadings(serial)
                        time.sleep(0.01)
                        hipOutputAngleCurrent = hipOutputAngleDesired
                        kneeOutputAngleCurrent = kneeOutputAngleDesired
                        if offsetCalibration:
                                globalStartTime = time.time()

                loopStartTime = time.time()
                elapsedTime = loopStartTime - globalStartTime
                timeSteps.append(elapsedTime)

                ######<<<<<< MAIN LOOP >>>>>>######
                hipRotorAngleDesired, kneeRotorAngleDesired = getRotorAngleRad(hipOutputAngleDesired - hipOffset), getRotorAngleRad(kneeOutputAngleDesired - kneeOffset)
                kpRotorHip, kdRotorHip, kpRotorKnee, kdRotorKnee = chooseRotorGains(crouching)

                ###<<< KNEE >>>###
                cmd.id = id.hip
                cmd.kp = kpRotorHip  # proportional or position term. i.e. stiffness
                cmd.kd = kdRotorHip  # derivative or velocity term, i.e damping
                cmd.q = hipRotorAngleDesired  # angle, radians
                cmd.dq = 0.0  # angular velocity, radians/s
                cmd.tau = hipTau  # rotor feedforward torque
                hipRcvStart = time.time()
                while not serial.sendRecv(cmd, data):
                        hipCommsFail += 1
                        print(f'Waiting for Hip motor to respond. Response lost {hipCommsFail} times out of {hipCommsFail + hipCommsSuccess}! {100 * hipCommsFail / (hipCommsFail + hipCommsSuccess):.2f}% failure rate."')
                hipOutputAngleCurrent = getOutputAngleDeg(data.q) + hipOffset
                hipCommsSuccess += 1

                hipSendRcvLength = time.time() - hipRcvStart

                hipTorque = calculateOutputTorque(kpRotorHip, hipRotorAngleDesired, data.q, kdRotorHip, 0.0, data.dq, hipTau)
                outputData(serial, id.hip, data.q, hipOffset, data.dq, hipTorque, data.temp, data.merror)
                hipOutputAngles.append(hipOutputAngleCurrent), hipCommandAngles.append(hipOutputAngleDesired), hipOutputTorque.append(hipTorque)

                ###<<< KNEE >>>###
                cmd.id = id.knee
                cmd.kp = kpRotorKnee # proportional or position term. i.e. stiffness
                cmd.kd = kdRotorKnee  # derivative or velocity term, i.e damping
                cmd.q = kneeRotorAngleDesired  # angle, radians
                cmd.dq = 0.0  # angular velocity, radians/s
                cmd.tau = kneeTau  # rotor feedforward torque
                kneeRcvStart = time.time()
                while not serial.sendRecv(cmd, data):
                        kneeCommsFail += 1
                        print(f'Waiting for Knee motor to respond. Response lost {kneeCommsFail} times out of {kneeCommsFail + kneeCommsSuccess}! " f"{100 * kneeCommsFail / (kneeCommsFail + kneeCommsSuccess):.2f}% failure rate."')
                kneeOutputAngleCurrent = getOutputAngleDeg(data.q) + kneeOffset
                kneeCommsSuccess += 1

                kneeSendRcvLength = time.time() - kneeRcvStart

                kneeTorque = calculateOutputTorque(kpRotorKnee, kneeRotorAngleDesired, data.q, kdRotorKnee, 0.0, data.dq, kneeTau)
                outputData(serial, id.knee, data.q, kneeOffset, data.dq, kneeTorque, data.temp, data.merror)
                kneeOutputAngles.append(kneeOutputAngleCurrent), kneeCommandAngles.append(kneeOutputAngleDesired), kneeOutputTorque.append(kneeTorque)

                ###<<< CROUCHING CONTROL >>>###
                crouchHeightDesiredNew = getCrouchCommand(pygame.event.get(),crouchHeightDesiredNew, crouchIncrement)
                hipOutputAngleDesired, kneeOutputAngleDesired, crouchHeightDesiredPrev, crouching = crouchControl(hipOutputAngleCurrent,kneeOutputAngleCurrent,crouchHeightDesiredPrev,crouchHeightDesiredNew,crouchDuration,crouching)

                ###<<< LOOP TIMING >>>###
                loopTime = time.time() - loopStartTime
                print(f"Loop Time: {loopTime}\n")
                print(f"Hip Send & Receive Timing: {hipSendRcvLength}\n") # >>> TAKES THE MOST TIME
                print(f"Knee Send & Receive Timing: {kneeSendRcvLength}\n")  # >>> TAKES THE MOST TIME

except KeyboardInterrupt:
        ### Command everything to 0?
        print("\nLoop stopped by user. Saving figure...")
        try:
                plotAndSaveLegData(timeSteps,hipOutputAngles,kneeOutputAngles,hipCommandAngles,kneeCommandAngles, hipOutputTorque, kneeOutputTorque, crouchDuration)
                print(f"Error encountered while saving figure: {e}")
        finally:
                sys.exit(0)  # Ensure clean exit
