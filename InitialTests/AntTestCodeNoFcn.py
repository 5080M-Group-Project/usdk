import time
import sys
import numpy as np
sys.path.append('../lib')
from unitree_actuator_sdk import *

serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()
gearRatio = queryGearRatio(MotorType.A1)



# Initialize Hip Motor
kpOutHip, kdOutHip, kpRotorHip, kdRotorHip = 25, 0.6, 0.0, 0.0
kpRotorHip = (kpOutHip / (gearRatio * gearRatio)) / 26.07
kdRotorHip = (kdOutHip / (gearRatio * gearRatio)) / 100.0
data.motorType = MotorType.A1
cmd.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
cmd.id    = 0;
cmd.kp    = 0.0; #proportional or position term. i.e. stiffness
cmd.kd    = 0.0; #derivative or velocity term, i.e damping
cmd.q     = 0.0; #angle, radians
cmd.dq    = 0.0; #angular velocity, radians/s
cmd.tau   = 0.0; #rotor feedforward torque
serial.sendRecv(cmd, data)
hipAngleOutputInitial = (data.q / gearRatio) * (180 / np.pi)

# Initialize Knee Motor
kpOutKnee, kdOutKnee, kpRotorKnee, kdRotorKnee = 25, 0.6, 0.0, 0.0
kpRotorKnee = (kpOutKnee / (gearRatio * gearRatio)) / 26.07
kdRotorKnee = (kdOutKnee / (gearRatio * gearRatio)) / 100.0

data.motorType = MotorType.A1
cmd.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
cmd.id    = 1;
cmd.kp    = 0.0; #proportional or position term. i.e. stiffness
cmd.kd    = 0.0; #derivative or velocity term, i.e damping
cmd.q     = 0.0; #angle, radians
cmd.dq    = 0.0; #angular velocity, radians/s
cmd.tau   = 0.0; #rotor feedforward torque
serial.sendRecv(cmd, data)
kneeAngleOutputInitial = (data.q / gearRatio) * (180 / np.pi)

# Initialize Wheel Motor
kpOutWheel, kdOutWheel, kpRotorWheel, kdRotorWheel = 0.0, 2, 0.0, 0.0
kpRotorWheel = (kpOutWheel / (gearRatio * gearRatio)) / 26.07
kdRotorWheel = (kdOutWheel / (gearRatio * gearRatio)) / 100.0

data.motorType = MotorType.A1
cmd.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
cmd.id    = 2;
cmd.kp    = 0.0; #proportional or position term. i.e. stiffness
cmd.kd    = 0.0; #derivative or velocity term, i.e damping
cmd.q     = 0.0; #angle, radians
cmd.dq    = 0.0; #angular velocity, radians/s
cmd.tau   = 0.0; #rotor feedforward torque
serial.sendRecv(cmd, data)
wheeAngleOutputInitial = (data.q / gearRatio) * (180 / np.pi)


#Initialize Loop Variables
torque = 0.0
hipRotorAngleDesired = 0.0
kneeRotorAngleDesired = 0.0
wheelRotorAngularVelocityDesired = 0.0
hipTau = 0.0
kneeTau = 0.0

while True:
        # Hip Motor Control
        ######DETERMINING DESIRED ANGLE#######
        data.motorType = MotorType.A1
        cmd.motorType = MotorType.A1
        cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
        cmd.id    = 0;
        cmd.kp    = kpRotorHip; #proportional or position term. i.e. stiffness
        cmd.kd    = kdRotorHip; #derivative or velocity term, i.e damping
        cmd.q     = hipRotorAngleDesired; #angle, radians
        cmd.dq    = 0.0; #angular velocity, radians/s
        cmd.tau   = hipTau; #rotor feedforward torque
        serial.sendRecv(cmd, data)
        torque = hipTau + kpRotorHip * (hipRotorAngleDesired - data.q) + kdRotorHip * (0.0 - data.dq);
        std::cout << std::endl;
        std::cout <<"Hip Motor" << std::endl;
        std::cout << "Angle (rad): " << data.q / gearRatio << std::endl;
        std::cout << "Angular Velocity (rad/s): " << data.dq / gearRatio << std::endl;
        std::cout << "Torque (N.m): " << torque << std::endl;
        std::cout << "Temperature: " << data.temp << std::endl;
        std::cout << "ISSUE? " << data.merror << std::endl;
        std::cout << std::endl;

        # Knee Motor Control
        ######DETERMINING DESIRED ANGLE#######
        data.motorType = MotorType.A1
        cmd.motorType = MotorType.A1
        cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
        cmd.id    = 1;
        cmd.kp    = kpRotorKnee; #proportional or position term. i.e. stiffness
        cmd.kd    = kdRotorKnee; #derivative or velocity term, i.e damping
        cmd.q     = kneeRotorAngleDesired; #angle, radians
        cmd.dq    = 0.0; #angular velocity, radians/s
        cmd.tau   = kneeTau; #rotor feedforward torque
        serial.sendRecv(cmd, data)
        torque = kneeTau + kpRotorKnee * (KneeRotorAngleDesired - data.q) + kdRotorKnee * (0.0 - data.dq);
        std::cout << std::endl;
        std::cout << "Knee Motor" << std::endl;
        std::cout << "Angle (rad): " << data.q / gearRatio << std::endl;
        std::cout << "Angular Velocity (rad/s): " << data.dq / gearRatio << std::endl;
        std::cout << "Torque (N.m): " << torque << std::endl;
        std::cout << "Temperature: " << data.temp << std::endl;
        std::cout << "ISSUE? " << data.merror << std::endl;
        std::cout << std::endl;

        # Wheel Motor Control
        ######DETERMINING DESIRED ANGULAR VELOCITY#######
        data.motorType = MotorType.A1
        cmd.motorType = MotorType.A1
        cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
        cmd.id    = 2;
        cmd.kp    = 0.0; #proportional or position term. i.e. stiffness
        cmd.kd    = kdRotorWheel; #derivative or velocity term, i.e damping
        cmd.q     = 0.0; #angle, radians
        cmd.dq    = wheelRotorAngularVelocityDesired; #angular velocity, radians/s
        cmd.tau   = 0.0; #rotor feedforward torque
        serial.sendRecv(cmd, data)
        torque = 0.0 + 0.0 * (0.0 - data.q) + kdRotorWheel * (wheelRotorAngularVelocityDesired - data.dq);
        std::cout << std::endl;
        std::cout << "Wheel Motor" << std::endl;
        std::cout << "Angle (rad): " << data.q / gearRatio << std::endl;
        std::cout << "Angular Velocity (rad/s): " << data.dq / gearRatio << std::endl;
        std::cout << "Torque (N.m): " << torque << std::endl;
        std::cout << "Temperature: " << data.temp << std::endl;
        std::cout << "ISSUE? " << data.merror << std::endl;
        std::cout << std::endl;

        time.sleep(0.0002) # 200 us
