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
    thetaKnee = -(np.pi - beta) * (180.0 / np.pi)

    # Return the angles as a numpy array (vector)
    return np.array([thetaHip, thetaKnee])

# Function to convert output gains to rotor gains
def getRotorGains(kpOutput, kdOutput):
    kpRotor = (kpOutput / (gearRatio * gearRatio)) / 26.07
    kdRotor = (kdOutput / (gearRatio * gearRatio)) * 100.0
    return np.array([kpRotor, kdRotor])

# Function to get the current motor output angle in DEGREES from rotor in RAD
def getOutputAngleDeg(rotorAngle):
    return (rotorAngle / gearRatio) * (180 / np.pi)

# Function to compute the desired rotor angle in RADIANS from output in RAD
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
def outputData(MotorID, q, dq, torque, temp, merror, offset):
        motorLabel = id.getName(MotorID)

        print("\n")
        print(f"{motorLabel} Motor")
        print(f"Angle (Deg): {getOutputAngleDeg(q) + offset}")
        print(f"Angular Velocity (rad/s): {dq / gearRatio}")
        print(f"Torque (N.m): {torque}")
        print(f"Temperature: {temp}")
        print(f"ISSUE? {merror}")
        print("\n")

def crouchingMechanismDeg(crouchHeightCurrent, crouchHeightDesired):
    # Define step size (dt) inside the function
    dt = 0.0001  # Fraction of the distance to move per iteration (LERP step size)

    # Get current and desired joint angles
    thetaHipCurrent, thetaKneeCurrent = inverseKinematicsDeg(0.0, crouchHeightCurrent)
    thetaHipDesired, thetaKneeDesired = inverseKinematicsDeg(0.0, crouchHeightDesired)

    # Apply standard linear interpolation (LERP) using dt
    thetaHipNew = thetaHipCurrent + dt * (thetaHipDesired - thetaHipCurrent)
    thetaKneeNew = thetaKneeCurrent + dt * (thetaKneeDesired - thetaKneeCurrent)

    # Return the updated angles
    return thetaHipNew, thetaKneeNew

def forwardKinematicsDeg(thetaHip, thetaKnee):
    L1 = 0.165  # Length of link 1
    L2 = 0.165  # Length of link 2

    xKnee = L1*np.cos(thetaHip)
    yKnee = L1*np.sin(thetaHip)

    yWheel = yKnee + L2*np.sin(thetaKnee + thetaHip)
    xWheel = xKnee + L2*np.cos(thetaKnee + thetaHip)
    return xWheel, yWheel

