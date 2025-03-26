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


# Initialize Hip Motor
kpOutHip, kdOutHip = 2.5, 0.2
kpRotorHip, kdRotorHip = getRotorGains(kpOutHip, kdOutHip)
cmdActuator(id.hip,0.0,0.0,0.0,0.0,0.0)
hipAngleOutputInitial = getOutputAngleDeg(data.q)


# Initialize Knee Motor
kpOutKnee, kdOutKnee = 2.5, 0.2
kpRotorKnee, kdRotorKnee = getRotorGains(kpOutKnee, kdOutKnee)
cmdActuator(id.knee,0.0,0.0,0.0,0.0,0.0)
kneeAngleOutputInitial  = getOutputAngleDeg(data.q)


# Initialize Wheel Motor
kpOutWheel, kdOutWheel = 0.0, 2.0
kpRotorWheel, kdRotorWheel = getRotorGains(kpOutWheel, kdOutWheel)
cmdActuator(id.wheel,0.0,0.0,0.0,0.0,0.0)
wheeAngleOutputInitial  = getOutputAngleDeg(data.q)


#Initialize Loop Variables
sinCounter = 0.0
torque = 0.0
hipRotorAngleDesired, kneeRotorAngleDesired, wheelRotorAngularVelocityDesired = 0.0, 0.0, 0.0
hipTau, kneeTau, wheelTau = 0.0, 0.0, 0.0


#Inverse Kinematics
xdes = 0.0  # Desired x coordinate
ydes = 0.15  # Desired y coordinate
thetaHipIK, thetaKneeIK = inverseKinematicsDeg(xdes, ydes)

while True:
        sinCounter += 0.005

        # Hip Motor Control
        hipOutputAngleDesired = hipAngleOutputInitial - (90 - thetaHipIK) * np.sin(2 * np.pi * sinCounter)
        hipRotorAngleDesired = getRotorAngleRad(hipOutputAngleDesired)

        cmdActuator(id.hip, kpRotorHip, kdRotorHip, hipRotorAngleDesired, 0.0, hipTau)
        hipTorque = calculateOutputTorque(kpRotorHip, kdRotorHip, hipRotorAngleDesired,0.0, hipTau, data.q, data.dq) #kpRotor or kpOutput??
        outputData(id.hip,data.q,data.dq,torque,data.temp,data.merror)

        # Knee Motor Control
        kneeOutputAngleDesired = kneeAngleOutputInitial + (thetaKneeIK) * np.sin( 2 * np.pi * sinCounter)
        kneeRotorAngleDesired = getRotorAngleRad(kneeOutputAngleDesired)

        cmdActuator(id.knee, kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired, 0.0, kneeTau)
        kneeTorque = calculateOutputTorque(kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired,0.0, kneeTau, data.q, data.dq) #kpRotor or kpOutput??
        outputData(id.knee, data.q, data.dq, torque, data.temp, data.merror)


        # Wheel Motor Control
        ######DETERMINING DESIRED ANGULAR VELOCITY#######
        cmdActuator(id.wheel, 0.0, kdRotorWheel, 0.0, wheelRotorAngularVelocityDesired, wheelTau)
        wheelTorque = calculateOutputTorque(0.0, kdRotorWheel, 0.0, wheelRotorAngularVelocityDesired, wheelTau, data.q, data.dq) #kpRotor or kpOutput??
        outputData(id.wheel, data.q, data.dq, torque, data.temp, data.merror)

        time.sleep(0.0002) # 200 us
