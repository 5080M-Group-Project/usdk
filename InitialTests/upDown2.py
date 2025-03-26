import time
import sys
import numpy as np
sys.path.append('../lib')
from unitree_actuator_sdk import *

##

##### MOTOR INITIALISATION #####

#Setup Serial Comms
serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()

#Rotor Reduction Ratio
gearRatio = queryGearRatio(MotorType.A1)

# Initialize Hip Motor
kpOutHip, kdOutHip = 2.5, 0.2
kpRotorHip = (kpOutHip / (gearRatio * gearRatio)) / 26.07
kdRotorHip = (kdOutHip / (gearRatio * gearRatio)) * 100.0
data.motorType = MotorType.A1
cmd.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
cmd.id    = 0
cmd.kp    = 0.0 #proportional or position term. i.e. stiffness
cmd.kd    = 0.0 #derivative or velocity term, i.e damping
cmd.q     = 0.0 #angle, radians
cmd.dq    = 0.0 #angular velocity, radians/s
cmd.tau   = 0.0 #rotor feedforward torque
serial.sendRecv(cmd, data)
hipAngleOutputInitial = (data.q / gearRatio) * (180 / np.pi)

# Initialize Knee Motor
kpOutKnee, kdOutKnee = 2.5, 0.2
kpRotorKnee = (kpOutKnee / (gearRatio * gearRatio)) / 26.07
kdRotorKnee = (kdOutKnee / (gearRatio * gearRatio)) * 100.0
data.motorType = MotorType.A1
cmd.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
cmd.id    = 1
cmd.kp    = 0.0 #proportional or position term. i.e. stiffness
cmd.kd    = 0.0 #derivative or velocity term, i.e damping
cmd.q     = 0.0 #angle, radians
cmd.dq    = 0.0 #angular velocity, radians/s
cmd.tau   = 0.0 #rotor feedforward torque
serial.sendRecv(cmd, data)
kneeAngleOutputInitial = (data.q / gearRatio) * (180 / np.pi)

# Initialize Wheel Motor
kpOutWheel, kdOutWheel = 0.0, 2
kpRotorWheel = (kpOutWheel / (gearRatio * gearRatio)) / 26.07
kdRotorWheel = (kdOutWheel / (gearRatio * gearRatio)) * 100.0
data.motorType = MotorType.A1
cmd.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
cmd.id    = 2
cmd.kp    = 0.0 #proportional or position term. i.e. stiffness
cmd.kd    = 0.0 #derivative or velocity term, i.e damping
cmd.q     = 0.0 #angle, radians
cmd.dq    = 0.0 #angular velocity, radians/s
cmd.tau   = 0.0 #rotor feedforward torque
serial.sendRecv(cmd, data)
wheeAngleOutputInitial = (data.q / gearRatio) * (180 / np.pi)


##### INVERSE KINEMATICS #####
L1 = 0.165  # Example length 1
L2 = 0.165  # Example length 2
xdes = 0.0  # Desired x coordinate
ydes = 0.15  # Desired y coordinate

# Use ** for exponentiation in Python
beta = np.arccos((L1**2 + L2**2 - xdes**2 - ydes**2) / (2 * L1 * L2))
alpha = np.arccos((xdes**2 + ydes**2 + L1**2 - L2**2) / (2 * L1 * np.sqrt(xdes**2 + ydes**2)))
gamma = np.arctan2(ydes, xdes)  # atan2(y, x) is correct

# Correct multiplication (no dereferencing in Python)
thetaHip = (gamma - alpha) * (180.0 / np.pi)
thetaKnee = (np.pi - beta) * (180.0 / np.pi)


##### LOOP VARIABLE INITIALISATION #####
sinCounter = 0.0
torque = 0.0
hipRotorAngleDesired = 0.0
kneeRotorAngleDesired = 0.0
wheelRotorAngularVelocityDesired = 0.0
hipTau = 0.0
kneeTau = 0.0

while True:
        sinCounter += 0.005

        ##### MOTOR CONTROL #####

        # Hip Motor Control
        hipOutputAngleDesired = hipAngleOutputInitial - (90 - thetaHip) * np.sin(2 * np.pi * sinCounter)
        hipRotorAngleDesired = (hipOutputAngleDesired * (np.pi / 180)) * gearRatio

        data.motorType = MotorType.A1
        cmd.motorType = MotorType.A1
        cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
        cmd.id    = 0
        cmd.kp    = kpRotorHip #proportional or position term. i.e. stiffness
        cmd.kd    = kdRotorHip #derivative or velocity term, i.e damping
        cmd.q     = hipRotorAngleDesired #angle, radians
        cmd.dq    = 0.0 #angular velocity, radians/s
        cmd.tau   = hipTau #rotor feedforward torque
        serial.sendRecv(cmd, data)
        torque = hipTau + kpRotorHip * (hipRotorAngleDesired - data.q) + kdRotorHip * (0.0 - data.dq)
        print('\n')
        print("Hip Motor")
        print("Angle (rad): " + str(data.q / gearRatio))
        print("Angular Velocity (rad/s): " + str(data.dq / gearRatio))
        print("Torque (N.m): " + str(torque))
        print("Temperature: " + str(data.temp))
        print("ERROR?: " + str(data.merror))
        print('\n')

        # Knee Motor Control
        kneeOutputAngleDesired = kneeAngleOutputInitial + (thetaKnee) * np.sin( 2 * np.pi * sinCounter)
        kneeRotorAngleDesired = (kneeOutputAngleDesired * (np.pi / 180)) * gearRatio

        data.motorType = MotorType.A1
        cmd.motorType = MotorType.A1
        cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
        cmd.id    = 1
        cmd.kp    = kpRotorKnee #proportional or position term. i.e. stiffness
        cmd.kd    = kdRotorKnee #derivative or velocity term, i.e damping
        cmd.q     = kneeRotorAngleDesired #angle, radians
        cmd.dq    = 0.0 #angular velocity, radians/s
        cmd.tau   = kneeTau #rotor feedforward torque
        serial.sendRecv(cmd, data)
        torque = kneeTau + kpRotorKnee * (kneeRotorAngleDesired - data.q) + kdRotorKnee * (0.0 - data.dq)
        print('\n')
        print("Knee Motor")
        print("Angle (rad): " + str(data.q / gearRatio))
        print("Angular Velocity (rad/s): " + str(data.dq / gearRatio))
        print("Torque (N.m): " + str(torque))
        print("Temperature: " + str(data.temp))
        print("ERROR?: " + str(data.merror))
        print('\n')

        # Wheel Motor Control
        ######DETERMINING DESIRED ANGULAR VELOCITY#######
        data.motorType = MotorType.A1
        cmd.motorType = MotorType.A1
        cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
        cmd.id    = 2
        cmd.kp    = 0.0 #proportional or position term. i.e. stiffness
        cmd.kd    = kdRotorWheel #derivative or velocity term, i.e damping
        cmd.q     = 0.0 #angle, radians
        cmd.dq    = wheelRotorAngularVelocityDesired #angular velocity, radians/s
        cmd.tau   = 0.0 #rotor feedforward torque
        serial.sendRecv(cmd, data)
        torque = 0.0 + 0.0 * (0.0 - data.q) + kdRotorWheel * (wheelRotorAngularVelocityDesired - data.dq)
        print('\n')
        print("Wheel Motor")
        print("Angle (rad): " + str(data.q / gearRatio))
        print("Angular Velocity (rad/s): " + str(data.dq / gearRatio))
        print("Torque (N.m): " + str(torque))
        print("Temperature: " + str(data.temp))
        print("ERROR?: " + str(data.merror))
        print('\n')

        time.sleep(0.0002) # 200 us
