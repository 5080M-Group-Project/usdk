import time
import sys
import numpy as np

sys.path.append('../lib')
from unitree_actuator_sdk import *

serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()
gearRatio = queryGearRatio(MotorType.A1)

class id: # e.g. id.hip = 0, id.get('Hip') = 0
    # Static variables (class variables)
    hip = 0
    knee = 1
    wheel = 2

    @staticmethod
    def get(motorName):
        # Return the corresponding motor ID based on the motor name
        motorName = motorName.lower()
        if motorName == 'hip':
            return id.hip
        elif motorName == 'knee':
            return id.knee
        elif motorName == 'wheel':
            return id.wheel
        else:
            return None

    @staticmethod
    def getName(motorID):
        # Return motor name based on the motor ID value
        if motorID == id.hip:
            return 'Hip'
        elif motorID == id.knee:
            return 'Knee'
        elif motorID == id.wheel:
            return 'Wheel'
        else:
            return None

def inverseKinematicsDeg(xdes, ydes):
    # Define link lengths
    L1 = 0.165  # Length of link 1
    L2 = 0.165  # Length of link 2

    # Calculate intermediate angles
    beta = np.arccos((L1**2 + L2**2 - xdes**2 - ydes**2) / (2 * L1 * L2))
    alpha = np.arccos((xdes**2 + ydes**2 + L1**2 - L2**2) / (2 * L1 * np.sqrt(xdes**2 + ydes**2)))
    gamma = np.arctan2(ydes, xdes)

    # Calculate angles in degrees
    thetaHip = (gamma - alpha) * (180.0 / np.pi)
    thetaKnee = (np.pi - beta) * (180.0 / np.pi)

    # Return the angles as a numpy array (vector)
    return np.array([thetaHip, thetaKnee])

# Function to convert output gains to rotor gains
def getRotorGains(kpOutput, kdOutput):
    kpRotor = (kpOutput / (gearRatio * gearRatio)) / 26.07
    kdRotor = (kdOutput / (gearRatio * gearRatio)) * 100.0
    return np.array([kpRotor, kdRotor])

# Function to get the current motor output angle in DEGREES
def getOutputAngleDeg(rotorAngle):
    return (rotorAngle / gearRatio) * (180 / np.pi)

# Function to compute the desired rotor angle in RADIANS
def getRotorAngleRad(outputAngle):
    return (outputAngle * (np.pi / 180)) * gearRatio

# Function to send actuator commands
def cmdActuator(id, kp, kd, q, dq, tau):
    cmd.motorType = MotorType.A1
    data.motorType = MotorType.A1
    cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
    cmd.id = id
    cmd.kp = kp  # proportional or position term. i.e. stiffness
    cmd.kd = kd  # derivative or velocity term, i.e damping
    cmd.q = q    # angle, radians
    cmd.dq = dq  # angular velocity, radians/s
    cmd.tau = tau  # rotor feedforward torque

    serial.sendRecv(cmd, data)

# Function to compute output torque
def calculateOutputTorque(kp, kd, qDesired, dqDesired, tau, qCurrent, dqCurrent):
    return tau + kp * (qDesired - qCurrent) + kd * (dqDesired - dqCurrent)

# Function to output motor data
def outputData(id, q, dq, torque, temp, merror):
        motorLabel = id.getName(id)

        print("\n")
        print(f"{motorLabel} Motor")
        print(f"Angle (rad): {q / gearRatio}")
        print(f"Angular Velocity (rad/s): {dq / gearRatio}")
        print(f"Torque (N.m): {torque}")
        print(f"Temperature: {temp}")
        print(f"ISSUE? {merror}")
        print("\n")

