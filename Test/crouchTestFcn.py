import time
import sys
from typing import Any

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
kpOutHip, kdOutHip = 2.5, 0.2 ### IDEA: Modify throughout the loop i.e. when locking legs
kpRotorHip, kdRotorHip = getRotorGains(kpOutHip, kdOutHip)
cmdActuator(id.hip,0.0,0.0,0.0,0.0,0.0) #NEEDED?


# Initialize Knee Motor
kpOutKnee, kdOutKnee = 2.5, 0.2
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
loopTime = 0.0002

while True:
        while not offsetCalibration: ### & other
                hipOffset, kneeOffset, hipOutputAngleDesired, kneeOutputAngleDesired, offsetCalibration = calibrateJointReadings()
                time.sleep(loopTime)
                ### IDEA: Add position calibration

        # MAIN CONTROL LOOP

        hipRotorAngleDesired, kneeRotorAngleDesired = getRotorAngleRad(hipOutputAngleDesired - hipOffset), getRotorAngleRad(kneeOutputAngleDesired - kneeOffset)

        # Hip Motor Control
        cmdActuator(id.hip, kpRotorHip, kdRotorHip, hipRotorAngleDesired, 0.0, hipTau)
        hipTorque = calculateOutputTorque(kpRotorHip, kdRotorHip, hipRotorAngleDesired,0.0, hipTau, data.q, data.dq) #kpRotor or kpOutput??
        hipOutputAngleCurrent = getOutputAngleDeg(data.q) + hipOffset
        outputData(id.hip,hipOutputAngleCurrent,data.dq,torque,data.temp,data.merror)

        cmd.motorType = MotorType.A1
        data.motorType = MotorType.A1
        cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
        cmd.id = id.hip
        serial.sendRecv(cmd, data)
        print(f"\nHip Angle (Deg) NO OFFSET: {(data.q / gearRatio) * (180 / np.pi)}\n")


        time.sleep(loopTime)

        # Knee Motor Control
        cmdActuator(id.knee, kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired, 0.0, kneeTau)
        kneeTorque = calculateOutputTorque(kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired,0.0, kneeTau, data.q, data.dq) #kpRotor or kpOutput??
        kneeOutputAngleCurrent = getOutputAngleDeg(data.q) + kneeOffset
        outputData(id.knee, kneeOutputAngleCurrent, data.dq, torque, data.temp, data.merror)

        cmd.motorType = MotorType.A1
        data.motorType = MotorType.A1
        cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
        cmd.id = id.knee
        serial.sendRecv(cmd, data)
        print(f"\nKnee Angle (Deg) NO OFFSET: {(data.q / gearRatio) * (180 / np.pi)}\n")



        # Crouch Control
        crouchHeightDesired = 0.2  ## max = 0.33 / ### IDEA: in future, read signal from RC controller to change
        hipOutputAngleDesired, kneeOutputAngleDesired = crouchingMotion(crouchHeightDesired,hipOutputAngleCurrent,kneeOutputAngleCurrent)


        '''
        # Wheel Motor Control
        ######DETERMINING DESIRED ANGULAR VELOCITY FROM: POSITION, STEERING, AND BALANCE CONTROLLERS#######
        cmdActuator(id.wheel, 0.0, kdRotorWheel, 0.0, wheelRotorAngularVelocityDesired, wheelTau)
        wheelTorque = calculateOutputTorque(0.0, kdRotorWheel, 0.0, wheelRotorAngularVelocityDesired, wheelTau, data.q, data.dq) #kpRotor or kpOutput??
        outputData(id.wheel, data.q, data.dq, torque, data.temp, data.merror)
        #### When reading angle, can do q/2pi and remainder gives angle, then apply offset?
        '''

        time.sleep(loopTime) # 200 us ### IDEA: Link sleep time to dt in LERP of crouchingMechanism