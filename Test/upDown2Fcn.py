import time
import sys
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
kpOutHip, kdOutHip = 2.5, 0.2
kpRotorHip, kdRotorHip = getRotorGains(kpOutHip, kdOutHip)
cmdActuator(id.hip,0.0,0.0,0.0,0.0,0.0)



# Initialize Knee Motor
kpOutKnee, kdOutKnee = 2.5, 0.2
kpRotorKnee, kdRotorKnee = getRotorGains(kpOutKnee, kdOutKnee)
cmdActuator(id.knee,0.0,0.0,0.0,0.0,0.0)



# Initialize Wheel Motor
kpOutWheel, kdOutWheel = 0.0, 2.0
kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)
cmdActuator(id.wheel,0.0,0.0,0.0,0.0,0.0)
wheeAngularVelocityInitial  = getOutputAngleDeg(data.dq)


#Initialize Loop Variables
torque = 0.0
wheelAngularVelocityDesired, wheelRotorAngularVelocityDesired = 0.0, 0.0
hipTau, kneeTau, wheelTau = 0.0, 0.0, 0.0

offsetCalibration = False

while True:
        if offsetCalibration == False:
                data.motorType = MotorType.A1
                cmd.motorType = MotorType.A1
                cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
                cmd.id = 0
                serial.sendRecv(cmd, data)
                hipModelledInitial = 90
                hipAngleInitialRaw = getOutputAngleDeg(data.q)
                hipOffset = hipModelledInitial - hipAngleInitialRaw

                data.motorType = MotorType.A1
                cmd.motorType = MotorType.A1
                cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
                cmd.id = 1
                serial.sendRecv(cmd, data)
                kneeModelledInitial = 0.0
                kneeAngleInitialRaw = getOutputAngleDeg(data.q)
                kneeOffset = kneeModelledInitial - kneeAngleInitialRaw

                hipOutputAngleDesired, kneeOutputAngleDesired = hipAngleInitialRaw + hipOffset, kneeAngleInitialRaw + kneeOffset
                offsetCalibration = True
        else:
                hipRotorAngleDesired, kneeRotorAngleDesired = getRotorAngleRad(hipOutputAngleDesired - hipOffset), getRotorAngleRad(kneeOutputAngleDesired - kneeOffset)
                # Hip Motor Control
                cmdActuator(id.hip, kpRotorHip, kdRotorHip, hipRotorAngleDesired, 0.0, hipTau)
                hipTorque = calculateOutputTorque(kpRotorHip, kdRotorHip, hipRotorAngleDesired,0.0, hipTau, data.q, data.dq) #kpRotor or kpOutput??
                outputData(id.hip,data.q,data.dq,torque,data.temp,data.merror,hipOffset)
                hipOutputAngleCurrent = getOutputAngleDeg(data.q) + hipOffset

                # Knee Motor Control
                cmdActuator(id.knee, kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired, 0.0, kneeTau)
                kneeTorque = calculateOutputTorque(kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired,0.0, kneeTau, data.q, data.dq) #kpRotor or kpOutput??
                outputData(id.knee, data.q, data.dq, torque, data.temp, data.merror,kneeOffset)
                kneeOutputAngleCurrent = getOutputAngleDeg(data.q) + kneeOffset

                #Crouch Control
                crouchHeightDesired = 0.2  ## max = 0.33 / in future, read signal from RC controller to change
                crouchThreshold = 0.001  # m

                xWheelCurrent, crouchHeightCurrent = forwardKinematicsDeg(hipOutputAngleCurrent, kneeOutputAngleCurrent)
                crouchHeightError = abs(crouchHeightDesired - crouchHeightCurrent)
                if crouchHeightError > crouchThreshold:
                        hipOutputAngleDesired , kneeOutputAngleDesired = crouchingMechanismDeg(crouchHeightCurrent,crouchHeightDesired)
                        print(f"Adjusting Crouch Height - Current: {crouchHeightCurrent:.3f}, Desired: {crouchHeightDesired:.3f}")
                        print(f"Hip Angle - Current: {hipOutputAngleCurrent:.3f}, Desired: {hipOutputAngleDesired:.3f}")
                        print(f"Knee Angle - Current: {kneeOutputAngleCurrent:.3f}, Desired: {kneeOutputAngleDesired:.3f}")
                else:
                        hipOutputAngleDesired, kneeOutputAngleDesired = hipOutputAngleCurrent, kneeOutputAngleCurrent
                        print("Correct crouch height. Legs Fixed")

                # Wheel Motor Control
                ######DETERMINING DESIRED ANGULAR VELOCITY FROM: POSITION, STEERING, AND BALANCE CONTROLLERS#######
                cmdActuator(id.wheel, 0.0, kdRotorWheel, 0.0, wheelRotorAngularVelocityDesired, wheelTau)
                wheelTorque = calculateOutputTorque(0.0, kdRotorWheel, 0.0, wheelRotorAngularVelocityDesired, wheelTau, data.q, data.dq) #kpRotor or kpOutput??
                outputData(id.wheel, data.q, data.dq, torque, data.temp, data.merror, 0.0)
                #### When reading angle, can do q/2pi and remainder gives angle, then apply offset?
                time.sleep(0.0002) # 200 us
