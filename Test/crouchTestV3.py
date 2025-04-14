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
gearRatio = queryGearRatio(MotorType.A1)

cmd.motorType = MotorType.A1
data.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)

pygame.init()
screen = pygame.display.set_mode((400, 300))
############################

##### NOTE 1: All rotor angles in RAD, all output angles in DEG########
##### NOTE 2: Whenever reading angles +offset, whenever commanding -offset. Offset in DEG######

# Initialize Motor Gains - add to .h file or equivalent

#Initialize Loop Variables
torque = 0.0
hipOutputAngleDesired, kneeOutputAngleDesired = 0.0, 0.0
kpRotorHip, kdRotorHip = 0.0, 0.0
kpRotorKnee, kdRotorKnee = 0.0, 0.0

hipTau, kneeTau = 0.0, 0.0
hipOffset, kneeOffset = 0.0, 0.0

hipOutputAngleCurrent, kneeOutputAngleCurrent = 0.0, 0.0

offsetCalibration, positionCalibration = False, False
sleepTime = 0.1

crouching = False
crouchHeightDesiredPrev = crouchHeightMax
crouchHeightDesiredNew = 0.75*crouchHeightMax
crouchDuration = 1.0 #### scale by the distance required?
crouchIncrement = 0.25*crouchHeightMax

hipCommsSuccess, hipCommsFail, kneeCommsSuccess, kneeCommsFail = 0, 0, 0, 0

# Data storage for plotting
hipOutputAngles, hipCommandAngles, kneeOutputAngles, kneeCommandAngles, timeSteps = [], [], [], [], []

globalStartTime = time.time()

try:
        while True:
                while not offsetCalibration: ### & other
                        hipOffset, kneeOffset, hipOutputAngleDesired, kneeOutputAngleDesired, offsetCalibration = calibrateJointReadings()
                        time.sleep(sleepTime)
                        hipOutputAngleCurrent = hipOutputAngleDesired
                        kneeOutputAngleCurrent = kneeOutputAngleDesired
                        ### IDEA: Add position calibration
                        #globalStartTime = time.time()


                startTime = time.time()
                elapsedTime = startTime - globalStartTime
                timeSteps.append(elapsedTime)

                # MAIN CONTROL LOOP
                hipRotorAngleDesired, kneeRotorAngleDesired = getRotorAngleRad(hipOutputAngleDesired - hipOffset), getRotorAngleRad(kneeOutputAngleDesired - kneeOffset)
                kpRotorHip, kdRotorHip, kpRotorKnee, kdRotorKnee = chooseRotorGains(crouching)

                # Hip Motor Control
                #cmdActuator(id.hip, kpRotorHip, kdRotorHip, hipRotorAngleDesired, 0.0, hipTau)
                cmd.id = id.hip
                cmd.kp = kpRotorHip  # proportional or position term. i.e. stiffness
                cmd.kd = kdRotorHip  # derivative or velocity term, i.e damping
                cmd.q = hipRotorAngleDesired  # angle, radians
                cmd.dq = 0.0  # angular velocity, radians/s
                cmd.tau = hipTau  # rotor feedforward torque
                hipRcvStart = time.time()
                if serial.sendRecv(cmd, data):
                        hipOutputAngleCurrent = getOutputAngleDeg(data.q) + hipOffset
                        hipCommsSuccess += 1
                else:
                        hipOutputAngleCurrent = hipOutputAngleCurrent
                        hipCommsFail += 1
                        print(f"[WARNING] Hip Motor (ID {id.hip}) lost response {hipCommsFail} times out of {hipCommsFail + hipCommsSuccess}! " f"{100 * hipCommsFail / (hipCommsFail + hipCommsSuccess):.2f}% failure rate.")
                hipSendRcvLength = time.time() - hipRcvStart
                hipTorque = calculateOutputTorque(kpRotorHip, kdRotorHip, hipRotorAngleDesired,0.0, hipTau, data.q, data.dq) #kpRotor or kpOutput??
                outputData(id.hip,hipOutputAngleCurrent,data.dq,hipTorque,data.temp,data.merror)
                hipOutputAngles.append(hipOutputAngleCurrent), hipCommandAngles.append(hipOutputAngleDesired)
                kneeTimingBegin = time.time()

                # Knee Motor Control
                #cmdActuator(id.knee, kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired, 0.0, kneeTau)
                cmd.id = id.knee
                cmd.kp = kpRotorKnee # proportional or position term. i.e. stiffness
                cmd.kd = kdRotorKnee  # derivative or velocity term, i.e damping
                cmd.q = kneeRotorAngleDesired  # angle, radians
                cmd.dq = 0.0  # angular velocity, radians/s
                cmd.tau = kneeTau  # rotor feedforward torque
                kneeRcvStart = time.time()
                serial.sendRecv(cmd, data)
                if serial.sendRecv(cmd, data):
                        kneeOutputAngleCurrent = getOutputAngleDeg(data.q) + kneeOffset
                        kneeCommsSuccess += 1
                else:
                        kneeOutputAngleCurrent = kneeOutputAngleCurrent
                        kneeCommsFail += 1
                        print(f"[WARNING] Knee Motor (ID {id.knee}) lost response {kneeCommsFail} times out of {kneeCommsFail + kneeCommsSuccess}! " f"{100 * kneeCommsFail / (kneeCommsFail + kneeCommsSuccess):.2f}% failure rate.")
                kneeSendRcvLength = time.time() - kneeRcvStart
                kneeTorque = calculateOutputTorque(kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired,0.0, kneeTau, data.q, data.dq) #kpRotor or kpOutput??
                outputData(id.knee, kneeOutputAngleCurrent, data.dq, kneeTorque, data.temp, data.merror)
                kneeOutputAngles.append(kneeOutputAngleCurrent), kneeCommandAngles.append(kneeOutputAngleDesired)

                # Crouch Control
                crouchHeightDesiredNew = getNewCrouchHeight(pygame.event.get(),crouchHeightDesiredNew,crouchIncrement)

                hipOutputAngleDesired, kneeOutputAngleDesired, crouchHeightDesiredPrev, crouching = crouchControl(hipOutputAngleCurrent,kneeOutputAngleCurrent,crouchHeightDesiredPrev,crouchHeightDesiredNew,crouching)
                '''
                xWheel,yWheel = forwardKinematicsDeg(hipOutputAngleCurrent, kneeOutputAngleCurrent)
                crouchHeightCurrent = abs(yWheel)
                #crouchThreshold = (0.1 / 100) * 0.33
                startCrouching = (crouchHeightDesiredNew != crouchHeightDesiredPrev)
                #changeParameters = abs(crouchHeightCurrent - crouchHeightDesiredNew) < crouchThreshold
                # hipOutputAngleDesired, kneeOutputAngleDesired = crouchingMotion2(crouchHeightDesired,hipOutputAngleCurrent,kneeOutputAngleCurrent,sleepTime*2, 2.0)

                if startCrouching and not crouching:
                        hipCrouchAngleDesired, kneeCrouchAngleDesired  = inverseKinematicsDeg(0.0, -crouchHeightDesiredNew, 'front')
                        hipCrouchAngleStart, kneeCrouchAngleStart = hipOutputAngleCurrent, kneeOutputAngleCurrent
                        crouchStartTime = time.time()
                        crouching = True
                        stopCrouching = False
                elif crouching:
                        dt = time.time() - crouchStartTime
                        stopCrouching = dt >= crouchDuration
                        if stopCrouching:
                                hipOutputAngleDesired, kneeOutputAngleDesired = hipCrouchAngleDesired, kneeCrouchAngleDesired
                                crouching = False
                                crouchHeightDesiredPrev = crouchHeightDesiredNew
                        else:
                                hipOutputAngleDesired = getLinearInterpolationAngle(hipCrouchAngleStart, hipCrouchAngleDesired, crouchDuration, dt)
                                kneeOutputAngleDesired = getLinearInterpolationAngle(kneeCrouchAngleStart, kneeCrouchAngleDesired, crouchDuration, dt)
                                print(f"\nAdjusting Crouch Height - Current: {crouchHeightCurrent:.3f}, Desired: {crouchHeightDesiredNew:.3f}")
                else:
                        hipOutputAngleDesired, kneeOutputAngleDesired = hipCrouchAngleDesired, kneeCrouchAngleDesired
                        crouchHeightDesiredPrev = crouchHeightDesiredNew
                        print("\nCrouch Height Fixed\n")
                '''

                loopTime = time.time() - startTime
                print(f"Loop Time: {loopTime}\n")
                print(f"Hip Send & Receive Timing: {hipSendRcvLength}\n") # >>> TAKES THE MOST TIME
                print(f"Knee Send & Receive Timing: {kneeSendRcvLength}\n")  # >>> TAKES THE MOST TIME
                #time.sleep(sleepTime - loopTime)  # 200 us ### IDEA: Link sleep time to dt in LERP of crouchingMechanism
except KeyboardInterrupt:
        ### Command everything to 0
        print("\nLoop stopped by user. Saving figure...")
        try:
                plotFigure(timeSteps,hipOutputAngles,kneeOutputAngles,hipCommandAngles,kneeCommandAngles, crouchDuration, kpOutHipMoving, kdOutHipMoving)
                print(f"Error encountered while saving figure: {e}")
        finally:
                sys.exit(0)  # Ensure clean exit
