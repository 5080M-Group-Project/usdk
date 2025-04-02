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

# Initialize Hip Motor
kpOutHip, kdOutHip = 5.0, 0.2 ### IDEA: Modify throughout the loop i.e. when locking legs
kpRotorHip, kdRotorHip = getRotorGains(kpOutHip, kdOutHip)
cmdActuator(id.hip,0.0,0.0,0.0,0.0,0.0) #NEEDED?


# Initialize Knee Motor
kpOutKnee, kdOutKnee = 5.0, 0.2
kpRotorKnee, kdRotorKnee = getRotorGains(kpOutKnee, kdOutKnee)
cmdActuator(id.knee,0.0,0.0,0.0,0.0,0.0)


# Initialize Wheel Motor
kpOutWheel, kdOutWheel = 0.0, 2.0
kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)
cmdActuator(id.wheel,0.0,0.0,0.0,0.0,0.0)

wheeAngularVelocityInitial  = getOutputAngleDeg(data.dq) #NEEDED?

#Initialize Loop Variables
torque = 0.0
hipOutputAngleDesired, kneeOutputAngleDesired = 0.0, 0.0
wheelOutputAngularVelocityDesired, wheelRotorAngularVelocityDesired = 0.0, 0.0
hipTau, kneeTau, wheelTau = 0.0, 0.0, 0.0
hipOffset, kneeOffset = 0.0, 0.0

offsetCalibration = False
sleepTime = 0.02

crouching = False
crouchHeightDesiredPrev = 0.33

# Data storage for plotting
hipOutputAngles = []
kneeOutputAngles = []
hipCommandAngles = []
kneeCommandAngles = []
timeSteps = []

plt.ion()
fig, ax = plt.subplots()
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angle (deg)')
ax.set_title('Hip and Knee Angles Over Time')
line1, = ax.plot([], [], label='Hip Output Angles')
line2, = ax.plot([], [], label='Knee Output Angles')
line3, = ax.plot([], [], label='Hip Command Angle')
line4, = ax.plot([], [], label='Knee Command Angle')
ax.legend()
ax.grid()

globalStartTime = time.time()

while True:
        while not offsetCalibration: ### & other
                hipOffset, kneeOffset, hipOutputAngleDesired, kneeOutputAngleDesired, offsetCalibration = calibrateJointReadings()
                time.sleep(sleepTime)
                ### IDEA: Add position calibration
                #globalStartTime = time.time()

        # MAIN CONTROL LOOP
        startTime = time.time()
        elapsedTime = startTime - globalStartTime
        timeSteps.append(elapsedTime)

        hipRotorAngleDesired, kneeRotorAngleDesired = getRotorAngleRad(hipOutputAngleDesired - hipOffset), getRotorAngleRad(kneeOutputAngleDesired - kneeOffset)

        # Hip Motor Control
        cmdActuator(id.hip, kpRotorHip, kdRotorHip, hipRotorAngleDesired, 0.0, hipTau)
        hipTorque = calculateOutputTorque(kpRotorHip, kdRotorHip, hipRotorAngleDesired,0.0, hipTau, data.q, data.dq) #kpRotor or kpOutput??
        hipOutputAngleCurrent = getOutputAngleDeg(data.q) + hipOffset
        outputData(id.hip,hipOutputAngleCurrent,data.dq,torque,data.temp,data.merror)

        hipOutputAngles.append(hipOutputAngleCurrent)
        hipCommandAngles.append(hipOutputAngleDesired)
        #time.sleep(sleepTime)


        # Knee Motor Control
        cmdActuator(id.knee, kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired, 0.0, kneeTau)
        kneeTorque = calculateOutputTorque(kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired,0.0, kneeTau, data.q, data.dq) #kpRotor or kpOutput??
        kneeOutputAngleCurrent = getOutputAngleDeg(data.q) + kneeOffset
        outputData(id.knee, kneeOutputAngleCurrent, data.dq, torque, data.temp, data.merror)

        kneeOutputAngles.append(kneeOutputAngleCurrent)
        kneeCommandAngles.append(kneeOutputAngleDesired)

        # Crouch Control
        crouchHeightDesiredNew = 0.2  ## max = 0.33 / ### IDEA: in future, read signal from RC controller to change
        #hipOutputAngleDesired, kneeOutputAngleDesired = crouchingMotion2(crouchHeightDesired,hipOutputAngleCurrent,kneeOutputAngleCurrent,sleepTime*2, 2.0)


        if (crouchHeightDesiredNew != crouchHeightDesiredPrev) and not crouching:
                N = int(2.0 / sleepTime)  # Ensure N is an integer
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

        line1.set_data(timeSteps, hipOutputAngles)
        line2.set_data(timeSteps, kneeOutputAngles)
        line3.set_data(timeSteps, hipCommandAngles)
        line4.set_data(timeSteps, kneeCommandAngles)
        ax.relim()
        ax.autoscale_view()
        plt.pause(sleepTime)

        time.sleep(sleepTime) # 200 us ### IDEA: Link sleep time to dt in LERP of crouchingMechanism
        loopTime = startTime - time.time()
        print(f"Loop Time: {loopTime}\n")

