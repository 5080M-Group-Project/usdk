import time
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.append('../lib')
from unitree_actuator_sdk import *

serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()

gearRatio = queryGearRatio(MotorType.A1)

# Global variables for crouching state
crouching = False
count = 0
thetaHipVector = []
thetaKneeVector = []

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
    return float (outputAngle * (np.pi / 180)) * gearRatio

# Function to send actuator commands
def cmdActuator(id, kp, kd, q, dq, tau):
    '''
    cmd.motorType = MotorType.A1
    data.motorType = MotorType.A1
    cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
    '''
    cmd.id = id
    cmd.kp = kp  # proportional or position term. i.e. stiffness
    cmd.kd = kd  # derivative or velocity term, i.e damping
    cmd.q = q    # angle, radians
    cmd.dq = dq  # angular velocity, radians/s
    cmd.tau = tau  # rotor feedforward torque

    #serial.sendRecv(cmd, data)

def getOffset(motorID, modelledInitialAngle):
    """Calibrate a motor and return its offset and initial raw angle."""
    cmd.motorType = MotorType.A1
    data.motorType = MotorType.A1
    cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
    cmd.id = motorID
    serial.sendRecv(cmd, data)

    rawInitialAngle = getOutputAngleDeg(data.q)
    offset = modelledInitialAngle - rawInitialAngle  # Offset calculation integrated here
    #time.sleep(0.002)  # 200 us
    return offset, rawInitialAngle

def calibrateJointReadings():
    """Calibrate hip and knee motors and return offsets, initial angles, and calibration status."""
    hipOffset, hipAngleInitialRaw = getOffset(id.hip, -90)
    kneeOffset, kneeAngleInitialRaw = getOffset(id.knee, 0.0)


    # Check if the combined offset is within the acceptable range
    hipCalibration = 17 > hipAngleInitialRaw > 15
    kneeCalibration = 27 > kneeAngleInitialRaw > 26
    offsetCalibration = hipCalibration + kneeCalibration

    if offsetCalibration:
        print(f"\nAngle Offsets Calibrated - Hip: {hipOffset:.6f}, Knee: {kneeOffset:.6f}\n")
        hipOutputAngleDesired, kneeOutputAngleDesired = hipAngleInitialRaw + hipOffset, kneeAngleInitialRaw + kneeOffset
        # Return offsets, desired angles, and calibration status
        return hipOffset, kneeOffset, hipOutputAngleDesired, kneeOutputAngleDesired, True

    # Return offsets and status when calibration is not successful
    print(f"\nRaw Initial Angles - Hip: {hipAngleInitialRaw:.6f}, Knee: {kneeAngleInitialRaw:.6f}\n")
    return hipOffset, kneeOffset, None, None, False

# Function to compute output torque
def calculateOutputTorque(kp, kd, qDesired, dqDesired, tau, qCurrent, dqCurrent):
    return tau + kp * (qDesired - qCurrent) + kd * (dqDesired - dqCurrent)

# Function to output motor data
def outputData(motorID, qDeg, dqRads, torqueNm, temp, merror):
        '''
        cmd.motorType = MotorType.A1
        data.motorType = MotorType.A1
        cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
        cmd.id = motorID
        serial.sendRecv(cmd, data)
        '''

        motorLabel = id.getName(motorID)

        print("\n")
        print(f"{motorLabel} Motor")
        print(f"Angle (Deg): {qDeg}")
        print(f"Angular Velocity (rad/s): {dqRads / gearRatio}")
        print(f"Torque (N.m): {torqueNm}")
        print(f"Temperature: {temp}")
        print(f"ISSUE? {merror}")
        print("\n")

def forwardKinematicsDeg(thetaHip, thetaKnee):
    L1 = 0.165  # Length of link 1
    L2 = 0.165  # Length of link 2

    xKnee = L1*np.cos(thetaHip)
    yKnee = L1*np.sin(thetaHip)

    yWheel = yKnee + L2*np.sin(thetaKnee + thetaHip)
    xWheel = xKnee + L2*np.cos(thetaKnee + thetaHip)
    return xWheel, yWheel

def inverseKinematicsDeg(xdes, ydes, kneeDir):
    # Define link lengths
    L1 = 0.165  # Length of link 1
    L2 = 0.165  # Length of link 2
    if np.sqrt(xdes ** 2 + ydes ** 2) > L1 + L2:
        print("\nOut of range!\n")
        return None, None

    # Handle kneeDir input
    kneeDir = kneeDir.lower()
    if kneeDir not in ['front', 'back']:
        print("\nInvalid Knee Direction!\n")
        return None, None

    # Calculate intermediate angles
    beta = np.arccos((L1**2 + L2**2 - xdes**2 - ydes**2) / (2 * L1 * L2))
    alpha = np.arccos((xdes**2 + ydes**2 + L1**2 - L2**2) / (2 * L1 * np.sqrt(xdes**2 + ydes**2)))
    gamma = np.arctan2(ydes, xdes)

    if kneeDir == 'front':
        thetaHip = (gamma - alpha) * (180.0 / np.pi)
        thetaKnee = (np.pi - beta) * (180.0 / np.pi)
    elif kneeDir == 'back':
        thetaHip = (gamma + alpha) * (180.0 / np.pi)
        thetaKnee = (beta - np.pi) * (180.0 / np.pi)

    # Return angles as a tuple
    return thetaHip, thetaKnee



def crouchingMechanismDeg(crouchHeightCurrent, crouchHeightDesired):
    # Define step size (dt) inside the function
    dt = 0.00001  # Fraction of the distance to move per iteration (LERP step size)

    # Get current and desired joint angles
    thetaHipCurrent, thetaKneeCurrent = inverseKinematicsDeg(0.0, -crouchHeightCurrent,'front')
    thetaHipDesired, thetaKneeDesired = inverseKinematicsDeg(0.0, -crouchHeightDesired,'front')

    # Apply standard linear interpolation (LERP) using dt
    thetaHipNew = thetaHipCurrent + dt * (thetaHipDesired - thetaHipCurrent)
    thetaKneeNew = thetaKneeCurrent + dt * (thetaKneeDesired - thetaKneeCurrent)

    # Return the updated angles
    return thetaHipNew, thetaKneeNew

def crouchingMotionV1(crouchHeightDesired,hipOutputAngleCurrent,kneeOutputAngleCurrent):
    crouchThreshold = 0.001  # m

    xWheelCurrent, yWheelCurrent = forwardKinematicsDeg(hipOutputAngleCurrent, kneeOutputAngleCurrent)
    crouchHeightCurrent = abs(yWheelCurrent)
    crouchHeightError = abs(crouchHeightDesired - crouchHeightCurrent)

    if crouchHeightError > crouchThreshold:
        ### IDEA: use moving leg kp and kd for crouching motion??
        hipOutputAngleDesired, kneeOutputAngleDesired = crouchingMechanismDeg(crouchHeightCurrent, crouchHeightDesired)
        print("\n")
        print(f"Adjusting Crouch Height - Current: {crouchHeightCurrent:.3f}, Desired: {crouchHeightDesired:.3f}")
        ##print(f"Hip Angle - Current: {hipOutputAngleCurrent:.3f}, Desired: {hipOutputAngleDesired:.3f}")
        ##print(f"Knee Angle - Current: {kneeOutputAngleCurrent:.3f}, Desired: {kneeOutputAngleDesired:.3f}")
    else:
        ### IDEA: use fixed leg kp and kd for suspension??
        hipOutputAngleDesired, kneeOutputAngleDesired = hipOutputAngleCurrent, kneeOutputAngleCurrent
        print("\n")
        print("Correct crouch height. Legs Fixed")

    return hipOutputAngleDesired, kneeOutputAngleDesired



def plotFigure(timeSteps,hipOutputAngles,kneeOutputAngles,hipCommandAngles,kneeCommandAngles):
    # Ensure all lists have the same length
    min_length = min(len(timeSteps), len(hipOutputAngles), len(kneeOutputAngles), len(hipCommandAngles),
                     len(kneeCommandAngles))

    if min_length == 0:
        print("No data collected. Exiting without saving.")
        sys.exit(0)  # Exit safely if no data

    timeSteps = timeSteps[:min_length]
    hipOutputAngles = hipOutputAngles[:min_length]
    kneeOutputAngles = kneeOutputAngles[:min_length]
    hipCommandAngles = hipCommandAngles[:min_length]
    kneeCommandAngles = kneeCommandAngles[:min_length]

    # Plotting
    plt.figure()
    plt.plot(timeSteps, hipOutputAngles, label='Hip Output Angles')
    plt.plot(timeSteps, kneeOutputAngles, label='Knee Output Angles')
    plt.plot(timeSteps, hipCommandAngles, label='Hip Command Angle')
    plt.plot(timeSteps, kneeCommandAngles, label='Knee Command Angle')

    plt.xlabel('Time (s)')
    plt.ylabel('Angle (deg)')
    plt.title('Hip and Knee Angles Over Time')
    plt.legend()
    plt.grid()

    # Save the figure before exiting
    plt.savefig("JointAnglesOverTime.png", dpi=300)
    print("Figure saved as JointAngleOverTime.png")

    # Close the plot
    plt.close()

