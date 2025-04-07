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
############################

##### NOTE 1: All rotor angles in RAD, all output angles in DEG########
##### NOTE 2: Whenever reading angles +offset, whenever commanding -offset. Offset in DEG######

# Initialize Motor Gains - add to .h file or equivalent

# HIP
kpOutHipFixed, kdOutHipFixed = 10.0, 0.5 ### IDEA: Modify throughout the loop i.e. when locking legs
kpRotorHipFixed, kdRotorHipFixed = getRotorGains(kpOutHipFixed, kdOutHipFixed)

kpOutHipMoving, kdOutHipMoving = 5.0, 3.0 ### IDEA: Modify throughout the loop i.e. when locking legs
kpRotorHipMoving, kdRotorHipMoving = getRotorGains(kpOutHipMoving, kdOutHipMoving)

# KNEE
kpOutKneeFixed, kdOutKneeFixed = 10.0, 0.5
kpRotorKneeFixed, kdRotorKneeFixed = getRotorGains(kpOutKneeFixed, kdOutKneeFixed)

kpOutKneeMoving, kdOutKneeMoving = 5.0, 3.0
kpRotorKneeMoving, kdRotorKneeMoving = getRotorGains(kpOutKneeMoving, kdOutKneeMoving)

'''
cmd.motorType = MotorType.A1
data.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)

cmdActuator(id.hip,0.0,0.0,0.0,0.0,0.0) #NEEDED?

serial.sendRecv(cmd, data)



cmd.motorType = MotorType.A1
data.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)

cmdActuator(id.knee,0.0,0.0,0.0,0.0,0.0)

serial.sendRecv(cmd, data)
'''

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


crouching = False
startCrouching = False
stopCrouching = True
crouchHeightDesiredPrev = 0.33
crouchDuration = 2.0
crouchStartTime = 0.0

hipCrouchAngleStart, hipCrouchAngleDesired, kneeCrouchAngleStart, kneeCrouchAngleDesired = 0.0, 0.0, 0.0, 0.0


hipCommsSuccess = 0
hipCommsFail = 0
kneeCommsSuccess = 0
kneeCommsFail = 0

# Data storage for plotting
hipOutputAngles = []
kneeOutputAngles = []

hipCommandAngles = []
kneeCommandAngles = []

timeSteps = []

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

                # MAIN CONTROL LOOP
                startTime = time.time()
                elapsedTime = startTime - globalStartTime
                timeSteps.append(elapsedTime)

                hipRotorAngleDesired, kneeRotorAngleDesired = getRotorAngleRad(hipOutputAngleDesired - hipOffset), getRotorAngleRad(kneeOutputAngleDesired - kneeOffset)

                # Hip Motor Control
                cmd.motorType = MotorType.A1
                data.motorType = MotorType.A1
                cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)

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


                time.sleep(sleepTime/100.0)


                # Knee Motor Control
                cmd.motorType = MotorType.A1
                data.motorType = MotorType.A1
                cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)

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
                        print(f"[WARNING] Knee Motor (ID {id.knee}) lost response {kneeCommsFail} times out of {kneeCommsFail + kneeCommsSuccess}! " f"{100 * kneeCommsFail / (kneeCommsFail + kneeCommsSuccess):.2f}% failure rate.")

                kneeTorque = calculateOutputTorque(kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired,0.0, kneeTau, data.q, data.dq) #kpRotor or kpOutput??
                outputData(id.knee, kneeOutputAngleCurrent, data.dq, torque, data.temp, data.merror)

                kneeOutputAngles.append(kneeOutputAngleCurrent), kneeCommandAngles.append(kneeOutputAngleDesired)

                # Crouch Control
                crouchTimingBegin = time.time()

                crouchHeightDesiredNew = 0.2  ## max = 0.33 / ### IDEA: in future, read signal from RC controller to change
                xWheel,yWheel = forwardKinematicsDeg(hipOutputAngleCurrent, kneeOutputAngleCurrent)
                crouchHeightCurrent = abs(yWheel)
                crouchThreshold = (0.1 / 100) * 0.33

                startCrouching = (crouchHeightDesiredNew != crouchHeightDesiredPrev)
                stopCrouching = (time.time() - crouchStartTime) >= crouchDuration 

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

                crouchTimingLength = time.time() - crouchTimingBegin
                print(f"Crouch Time: {crouchTimingLength}\n")

                preCrouchTimingLength = crouchTimingBegin - startTime
                print(f"Pre Crouch Time: {preCrouchTimingLength}\n")

                time.sleep(sleepTime - loopTime)  # 200 us ### IDEA: Link sleep time to dt in LERP of crouchingMechanism
except KeyboardInterrupt:
        ### Command everything to 0
        print("\nLoop stopped by user. Saving figure...")
        try:
                plotFigure(timeSteps,hipOutputAngles,kneeOutputAngles,hipCommandAngles,kneeCommandAngles, crouchDuration, kpOutHipMoving, kdOutHipMoving)
                print(f"Error encountered while saving figure: {e}")
        finally:
                sys.exit(0)  # Ensure clean exit
