import time
import sys
from typing import Any
import matplotlib.pyplot as plt
import numpy as np

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

# Initialize Hip Motor
kpOutHip, kdOutHip = 10.0, 3.0 ### IDEA: Modify throughout the loop i.e. when locking legs
kpRotorHip, kdRotorHip = getRotorGains(kpOutHip, kdOutHip)

# Initialize Knee Motor
kpOutKnee, kdOutKnee = 10.0, 3.0
kpRotorKnee, kdRotorKnee = getRotorGains(kpOutKnee, kdOutKnee)

'''
# Initialize Wheel Motor
kpOutWheel, kdOutWheel = 0.0, 2.0
kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)
'''

#Initialize Loop Variables
hipOutputAngleDesired,hipOutputAngleCurrent, hipTau, hipOffset  = 0.0, 0.0, 0.0, 0.0
kneeOutputAngleDesired, kneeOutputAngleCurrent, kneeTau, kneeOffset = 0.0, 0.0, 0.0, 0.0

wheelOutputAngularVelocityDesired, wheelRotorAngularVelocityDesired, wheelTau = 0.0, 0.0, 0.0


offsetCalibration = False
sleepTime = 0.1

crouching = False
crouchHeightDesiredPrev = 0.33
crouchTime = 2.0


hipCommsSuccess, hipCommsFail, kneeCommsSuccess, kneeCommsFail = 0, 0, 0, 0

# Data storage for plotting
hipOutputAngles, hipCommandAngles, kneeOutputAngles, kneeCommandAngles, timeSteps = [], [], [], [], []

globalStartTime = time.time()

try:
        while True:
                while not offsetCalibration: ### & other
                        hipOffset, kneeOffset, hipOutputAngleDesired, kneeOutputAngleDesired, offsetCalibration = calibrateJointReadings()
                        #hipOffset, kneeOffset, None, None, False
                        time.sleep(sleepTime)
                        hipOutputAngleCurrent = hipOutputAngleDesired
                        kneeOutputAngleCurrent = kneeOutputAngleDesired
                        ### IDEA: Add position calibration
                        #globalStartTime = time.time()

                        # wheeAngularVelocityInitial  = getOutputAngleDeg(data.dq) #NEEDED?

                # MAIN CONTROL LOOP
                startTime = time.time()
                elapsedTime = startTime - globalStartTime
                timeSteps.append(elapsedTime)

                hipRotorAngleDesired, kneeRotorAngleDesired = getRotorAngleRad(hipOutputAngleDesired - hipOffset), getRotorAngleRad(kneeOutputAngleDesired - kneeOffset)

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
                outputData(id.hip,hipOutputAngleCurrent,data.dq,hipTorque,data.temp,data.merror)

                hipOutputAngles.append(hipOutputAngleCurrent)
                hipCommandAngles.append(hipOutputAngleDesired)


                #time.sleep(sleepTime/100.0)


                # Knee Motor Control
                #cmdActuator(id.knee, kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired, 0.0, kneeTau)
                cmd.id = id.knee
                cmd.kp = kpRotorKnee # proportional or position term. i.e. stiffness
                cmd.kd = kdRotorKnee  # derivative or velocity term, i.e damping
                cmd.q = kneeRotorAngleDesired  # angle, radians
                cmd.dq = 0.0  # angular velocity, radians/s
                cmd.tau = kneeTau  # rotor feedforward torque
                if serial.sendRecv(cmd, data):
                        kneeOutputAngleCurrent = getOutputAngleDeg(data.q) + kneeOffset
                        kneeCommsSuccess += 1
                else:
                        kneeOutputAngleCurrent = kneeOutputAngleCurrent
                        kneeCommsFail += 1
                        print(f"[WARNING] Knee Motor (ID {id.knee}) lost response {kneeCommsFail} times out of {kneeCommsFail+kneeCommsSuccess}! " f"{100 * kneeCommsFail / (kneeCommsFail + kneeCommsSuccess):.2f}% failure rate.")

                kneeTorque = calculateOutputTorque(kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired,0.0, kneeTau, data.q, data.dq) #kpRotor or kpOutput??
                outputData(id.knee, kneeOutputAngleCurrent, data.dq, kneeTorque, data.temp, data.merror)

                kneeOutputAngles.append(kneeOutputAngleCurrent)
                kneeCommandAngles.append(kneeOutputAngleDesired)

                crouchTimingBegin = time.time()
                # Crouch Control
                crouchHeightDesiredNew = 0.2  ## max = 0.33 / ### IDEA: in future, read signal from RC controller to change
                #hipOutputAngleDesired, kneeOutputAngleDesired = crouchingMotion2(crouchHeightDesired,hipOutputAngleCurrent,kneeOutputAngleCurrent,sleepTime*2, 2.0)

                if (crouchHeightDesiredNew != crouchHeightDesiredPrev) and not crouching:
                        N = int(crouchTime / sleepTime)  # Ensure N is an integer
                        # Get current and desired joint angles
                        hipCrouchAngleDesired, kneeCrouchAngleDesired = inverseKinematicsDeg(0.0, -crouchHeightDesiredNew, 'front')
                        # Generate interpolation vectors
                        thetaHipVector = np.linspace(hipOutputAngleCurrent, hipCrouchAngleDesired, num=N)
                        thetaKneeVector = np.linspace(kneeOutputAngleDesired, kneeCrouchAngleDesired, num=N)
                        count = 0
                        crouching = True  # Enable crouching phase
                elif crouching and count < len(thetaHipVector):
                        hipOutputAngleDesired, kneeOutputAngleDesired = thetaHipVector[count], thetaKneeVector[count]
                        count += 1  # Increment step
                        print(f"\nCount: {(count)}\n")
                        # print(f"\nAdjusting Crouch Height - Current: {crouchHeightCurrent:.3f}, Desired: {crouchHeightDesired:.3f}")
                elif crouching and count >= len(thetaHipVector):
                        hipOutputAngleDesired, kneeOutputAngleDesired = hipCrouchAngleDesired, kneeCrouchAngleDesired
                        crouching = False
                        crouchHeightDesiredPrev = crouchHeightDesiredNew
                        print("\nCorrect crouch height. Legs Fixed\n")
                else:
                        hipOutputAngleDesired, kneeOutputAngleDesired = hipCrouchAngleDesired, kneeCrouchAngleDesired
                        crouchHeightDesiredPrev = crouchHeightDesiredNew
                        print("\nCorrect crouch height. Legs Fixed\n")

                loopTime = time.time() - startTime
                print(f"Loop Time: {loopTime}\n")

                crouchTimingLength = time.time() - crouchTimingBegin
                print(f"Crouch Time: {crouchTimingLength}\n")

                preCrouchTimingLength = crouchTimingBegin - startTime
                print(f"Pre Crouch Time: {preCrouchTimingLength}\n")

                time.sleep(sleepTime - loopTime)  # 200 us ### IDEA: Link sleep time to dt in LERP of crouchingMechanism



except KeyboardInterrupt:
        ### Command everything to zero
        print("\nLoop stopped by user. Saving figure...")
        try:
                plotFigure(timeSteps,hipOutputAngles,kneeOutputAngles,hipCommandAngles,kneeCommandAngles, crouchTime, kpOutHip, kdOutHip)
        except Exception as e:
                print(f"Error encountered while saving figure: {e}")
        finally:
                sys.exit(0)  # Ensure clean exit
