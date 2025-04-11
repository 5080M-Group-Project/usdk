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
############################

##### NOTE 1: All rotor angles in RAD, all output angles in DEG########
##### NOTE 2: Whenever reading angles +offset, whenever commanding -offset. Offset in DEG######

# Initialize Motor Gains - add to .h file or equivalent

# HIP
kpOutHipFixed, kdOutHipFixed = 20.0, 0.5 ### kp = 20, kd = 0.5
kpRotorHipFixed, kdRotorHipFixed = getRotorGains(kpOutHipFixed, kdOutHipFixed)

kpOutHipMoving, kdOutHipMoving = 15.0, 2.0 ### kp = 10, kd = 3.0
kpRotorHipMoving, kdRotorHipMoving = getRotorGains(kpOutHipMoving, kdOutHipMoving)

# KNEE
kpOutKneeFixed, kdOutKneeFixed = 20.0, 0.5
kpRotorKneeFixed, kdRotorKneeFixed = getRotorGains(kpOutKneeFixed, kdOutKneeFixed)

kpOutKneeMoving, kdOutKneeMoving = 15.0, 2.0
kpRotorKneeMoving, kdRotorKneeMoving = getRotorGains(kpOutKneeMoving, kdOutKneeMoving)

'''
# Initialize Wheel Motor
kpOutWheel, kdOutWheel = 0.0, 2.0
kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)

cmdActuator(id.wheel,0.0,0.0,0.0,0.0,0.0)

#wheeAngularVelocityInitial  = getOutputAngleDeg(data.dq) #NEEDED?
'''

#Initialize Loop Variables
torque = 0.0
hipOutputAngleDesired, kneeOutputAngleDesired = 0.0, 0.0
kpRotorHip, kdRotorHip = 0.0, 0.0
kpRotorKnee, kdRotorKnee = 0.0, 0.0

wheelOutputAngularVelocityDesired, wheelRotorAngularVelocityDesired = 0.0, 0.0
hipTau, kneeTau, wheelTau = 0.0, 0.0, 0.0
hipOffset, kneeOffset = 0.0, 0.0

hipOutputAngleCurrent, kneeOutputAngleCurrent = 0.0, 0.0

offsetCalibration = False
sleepTime = 0.1

crouching, startCrouching, stopCrouching = False, False, True
crouchHeightMax = 0.33
crouchIncrement = 0.1*crouchHeightMax
crouchHeightDesiredPrev = crouchHeightMax
crouchDuration = 3.0 #### scale by the distance required
crouchStartTime = 0.0

hipCrouchAngleStart, hipCrouchAngleDesired, kneeCrouchAngleStart, kneeCrouchAngleDesired = 0.0, 0.0, 0.0, 0.0

hipCommsSuccess, hipCommsFail, kneeCommsSuccess, kneeCommsFail = 0, 0, 0, 0

# Data storage for plotting
hipOutputAngles, hipCommandAngles, kneeOutputAngles, kneeCommandAngles, timeSteps = [], [], [], [], []

globalStartTime = time.time()

try:
        while True:
                startTime = time.time()
                elapsedTime = startTime - globalStartTime
                timeSteps.append(elapsedTime)

                while not offsetCalibration: ### & other
                        hipOffset, kneeOffset, hipOutputAngleDesired, kneeOutputAngleDesired, offsetCalibration = calibrateJointReadings()
                        #hipOffset, kneeOffset, None, None, False
                        time.sleep(sleepTime)
                        hipOutputAngleCurrent = hipOutputAngleDesired
                        kneeOutputAngleCurrent = kneeOutputAngleDesired
                        ### IDEA: Add position calibration
                        #globalStartTime = time.time()

                # MAIN CONTROL LOOP


                hipRotorAngleDesired, kneeRotorAngleDesired = getRotorAngleRad(hipOutputAngleDesired - hipOffset), getRotorAngleRad(kneeOutputAngleDesired - kneeOffset)

                hipTimingBegin = time.time()

                # Hip Motor Control
                #cmdActuator(id.hip, kpRotorHip, kdRotorHip, hipRotorAngleDesired, 0.0, hipTau)
                cmd.id = id.hip
                cmd.kp = kpRotorHip  # proportional or position term. i.e. stiffness
                cmd.kd = kdRotorHip  # derivative or velocity term, i.e damping
                cmd.q = hipRotorAngleDesired  # angle, radians
                cmd.dq = 0.0  # angular velocity, radians/s
                cmd.tau = hipTau  # rotor feedforward torque
                if serial.sendRecv(cmd, data):
                        hipOutputAngleCurrent = getOutputAngleDeg(data.q) + hipOffset
                        hipCommsSuccess += 1
                else:
                        hipOutputAngleCurrent = hipOutputAngleCurrent
                        hipCommsFail += 1
                        print(f"[WARNING] Hip Motor (ID {id.hip}) lost response {hipCommsFail} times out of {hipCommsFail + hipCommsSuccess}! " f"{100 * hipCommsFail / (hipCommsFail + hipCommsSuccess):.2f}% failure rate.")

                hipTorque = calculateOutputTorque(kpRotorHip, kdRotorHip, hipRotorAngleDesired,0.0, hipTau, data.q, data.dq) #kpRotor or kpOutput??
                outputData(id.hip,hipOutputAngleCurrent,data.dq,torque,data.temp,data.merror)

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

                kneeCommandEndTiming = time.time()
                kneeCommandLength = kneeCommandEndTiming - kneeTimingBegin

                serial.sendRecv(cmd, data)
                if serial.sendRecv(cmd, data):
                        kneeOutputAngleCurrent = getOutputAngleDeg(data.q) + kneeOffset
                        kneeCommsSuccess += 1
                else:
                        kneeOutputAngleCurrent = kneeOutputAngleCurrent
                        kneeCommsFail += 1
                        print(f"[WARNING] Knee Motor (ID {id.knee}) lost response {kneeCommsFail} times out of {kneeCommsFail + kneeCommsSuccess}! " f"{100 * kneeCommsFail / (kneeCommsFail + kneeCommsSuccess):.2f}% failure rate.")

                kneeSendRcvLength = time.time() - kneeCommandEndTiming

                kneeTorque = calculateOutputTorque(kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired,0.0, kneeTau, data.q, data.dq) #kpRotor or kpOutput??
                outputData(id.knee, kneeOutputAngleCurrent, data.dq, torque, data.temp, data.merror)

                kneeOutputAngles.append(kneeOutputAngleCurrent), kneeCommandAngles.append(kneeOutputAngleDesired)

                kneeCalcValuesLength = time.time() - kneeSendRcvLength - kneeCommandEndTiming

                # Crouch Control
                crouchTimingBegin = time.time()

                events = pygame.event.get()
                for event in events:
                        if event.type == pygame.KEYDOWN:
                                if event.key == pygame.K_UP:
                                        crouchHeightDesiredNew += crouchIncrement
                                if event.key == pygame.K_DOWN:
                                        crouchHeightDesiredNew -= crouchIncrement

                crouchHeightDesiredNew = max(0.1*crouchHeightMax, min(crouchHeightMax, crouchHeightDesiredNew))

                #crouchHeightDesiredNew = 0.25  ## max = 0.33 / ### IDEA: in future, read signal from RC controller to change

                xWheel,yWheel = forwardKinematicsDeg(hipOutputAngleCurrent, kneeOutputAngleCurrent)
                crouchHeightCurrent = abs(yWheel)
                crouchThreshold = (0.1 / 100) * 0.33

                startCrouching = (crouchHeightDesiredNew != crouchHeightDesiredPrev)
                stopCrouching = (time.time() - crouchStartTime) >= crouchDuration #stops based on command

                #changeParameters = abs(crouchHeightCurrent - crouchHeightDesiredNew) < crouchThreshold


                # hipOutputAngleDesired, kneeOutputAngleDesired = crouchingMotion2(crouchHeightDesired,hipOutputAngleCurrent,kneeOutputAngleCurrent,sleepTime*2, 2.0)


                if startCrouching and not crouching:
                        # Get current and desired joint angles
                        hipCrouchAngleDesired, kneeCrouchAngleDesired = inverseKinematicsDeg(0.0,-crouchHeightDesiredNew,'front')
                        hipCrouchAngleStart, kneeCrouchAngleStart = hipOutputAngleCurrent, kneeOutputAngleCurrent
                        crouchStartTime = time.time()
                        crouching = True  # Enable crouching phase
                        stopCrouching = False  # NEEDED?

                        kpRotorHip, kdRotorHip = kpRotorHipMoving, kdRotorHipMoving
                        kpRotorKnee, kdRotorKnee = kpRotorKneeMoving, kdRotorKneeMoving

                elif crouching and not stopCrouching:
                        hipOutputAngleDesired = getLinearInterpolationAngle(hipCrouchAngleStart, hipCrouchAngleDesired, crouchDuration, time.time() - crouchStartTime)
                        kneeOutputAngleDesired = getLinearInterpolationAngle(kneeCrouchAngleStart, kneeCrouchAngleDesired, crouchDuration, time.time() - crouchStartTime)

                        #####
                        kpRotorHip, kdRotorHip = kpRotorHipMoving, kdRotorHipMoving
                        kpRotorKnee, kdRotorKnee = kpRotorKneeMoving, kdRotorKneeMoving
                        #####

                        print(f"\nAdjusting Crouch Height - Current: {crouchHeightCurrent:.3f}, Desired: {crouchHeightDesiredNew:.3f}")

                elif crouching and stopCrouching:
                        hipOutputAngleDesired, kneeOutputAngleDesired = hipCrouchAngleDesired, kneeCrouchAngleDesired
                        crouching = False
                        crouchHeightDesiredPrev = crouchHeightDesiredNew
                else:
                        hipOutputAngleDesired, kneeOutputAngleDesired = hipCrouchAngleDesired, kneeCrouchAngleDesired
                        crouchHeightDesiredPrev = crouchHeightDesiredNew

                        #####
                        kpRotorHip, kdRotorHip = kpRotorHipFixed, kdRotorHipFixed
                        kpRotorKnee, kdRotorKnee = kpRotorKneeFixed, kdRotorKneeFixed
                        #####

                        print("\nCorrect crouch height. Legs Fixed\n")


                loopTime = time.time() - startTime
                print(f"Loop Time: {loopTime}\n")

                calibrationCheckTimingLength = hipTimingBegin - startTime
                print(f"Calibration Check Time: {calibrationCheckTimingLength}\n")

                hipTimingLength =  kneeTimingBegin - hipTimingBegin
                print(f"Hip Time: {hipTimingLength}\n")

                kneeTimingLength = crouchTimingBegin - kneeTimingBegin
                print(f"Knee Time: {kneeTimingLength}\n")

                print(f"Knee Command Timing: {kneeCommandLength}\n")

                print(f"Knee Send & Recieve Timing: {kneeSendRcvLength}\n") # >>> TAKES THE MOST TIME

                print(f"Knee Calcs Timing: {kneeCalcValuesLength}\n")

                crouchTimingLength = time.time() - crouchTimingBegin
                print(f"Crouch Time: {crouchTimingLength}\n")



                #time.sleep(sleepTime - loopTime)  # 200 us ### IDEA: Link sleep time to dt in LERP of crouchingMechanism
except KeyboardInterrupt:
        ### Command everything to 0
        print("\nLoop stopped by user. Saving figure...")
        try:
                plotFigure(timeSteps,hipOutputAngles,kneeOutputAngles,hipCommandAngles,kneeCommandAngles, crouchDuration, kpOutHipMoving, kdOutHipMoving)
                print(f"Error encountered while saving figure: {e}")
        finally:
                sys.exit(0)  # Ensure clean exit
