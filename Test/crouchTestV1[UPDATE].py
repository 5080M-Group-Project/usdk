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
sendCmdRcvData(id.hip, 0.0, 0.0, 0.0, 0.0, 0.0, )  #NEEDED?


# Initialize Knee Motor
kpOutKnee, kdOutKnee = 5.0, 0.2
kpRotorKnee, kdRotorKnee = getRotorGains(kpOutKnee, kdOutKnee)
sendCmdRcvData(id.knee, 0.0, 0.0, 0.0, 0.0, 0.0, )


# Initialize Wheel Motor
kpOutWheel, kdOutWheel = 0.0, 2.0
kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)
sendCmdRcvData(id.wheel, 0.0, 0.0, 0.0, 0.0, 0.0, )

wheeAngularVelocityInitial  = getOutputAngleDeg(data.dq) #NEEDED?

#Initialize Loop Variables
torque = 0.0
hipOutputAngleDesired, kneeOutputAngleDesired = 0.0, 0.0
wheelOutputAngularVelocityDesired, wheelRotorAngularVelocityDesired = 0.0, 0.0
hipTau, kneeTau, wheelTau = 0.0, 0.0, 0.0
hipOffset, kneeOffset = 0.0, 0.0

offsetCalibration = False
sleepTime = 0.002

while True:
        while not offsetCalibration: ### & other
                hipOffset, kneeOffset, hipOutputAngleDesired, kneeOutputAngleDesired, offsetCalibration = calibrateJointReadings()
                time.sleep(sleepTime)
                ### IDEA: Add position calibration

        # MAIN CONTROL LOOP

        hipRotorAngleDesired, kneeRotorAngleDesired = getRotorAngleRad(hipOutputAngleDesired - hipOffset), getRotorAngleRad(kneeOutputAngleDesired - kneeOffset)

        # Hip Motor Control

        sendCmdRcvData(id.hip, kpRotorHip, kdRotorHip, hipRotorAngleDesired, 0.0, hipTau, )
        hipTorque = calculateOutputTorque(kpRotorHip, kdRotorHip, hipRotorAngleDesired,0.0, hipTau, data.q, data.dq) #kpRotor or kpOutput??
        hipOutputAngleCurrent = getOutputAngleDeg(data.q) + hipOffset
        outputData(id.hip,hipOutputAngleCurrent,data.dq,torque,data.temp,data.merror)
        time.sleep(sleepTime)


        # Knee Motor Control
        sendCmdRcvData(id.knee, kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired, 0.0, kneeTau, )
        kneeTorque = calculateOutputTorque(kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired,0.0, kneeTau, data.q, data.dq) #kpRotor or kpOutput??
        kneeOutputAngleCurrent = getOutputAngleDeg(data.q) + kneeOffset
        outputData(id.knee, kneeOutputAngleCurrent, data.dq, torque, data.temp, data.merror)


        # Crouch Control
        crouchHeightDesired = 0.2  ## max = 0.33 / ### IDEA: in future, read signal from RC controller to change
        hipOutputAngleDesired, kneeOutputAngleDesired = crouchingMotionV1(crouchHeightDesired,hipOutputAngleCurrent,kneeOutputAngleCurrent)


        time.sleep(sleepTime) # 200 us ### IDEA: Link sleep time to dt in LERP of crouchingMechanism