import time
import sys
import os
import csv
import numpy as np
import matplotlib.pyplot as plt
import pygame

sys.path.append('../lib')
from unitree_actuator_sdk import *

serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()

gearRatio = queryGearRatio(MotorType.A1)

# Function to convert output gains to rotor gains
def getRotorGains(kpOutput, kdOutput):
    kpRotor = (kpOutput / (gearRatio * gearRatio)) / 26.07
    kdRotor = (kdOutput / (gearRatio * gearRatio)) * 100.0
    return np.array([kpRotor, kdRotor])

# HIP
kpOutHipFixed, kdOutHipFixed = 20.0, 0.5  ### kp = 20, kd = 0.5
kpRotorHipFixed, kdRotorHipFixed = getRotorGains(kpOutHipFixed, kdOutHipFixed)

kpOutHipMoving, kdOutHipMoving = 15.0, 2.0  ### kp = 10, kd = 3.0
kpRotorHipMoving, kdRotorHipMoving = getRotorGains(kpOutHipMoving, kdOutHipMoving)

# KNEE
kpOutKneeFixed, kdOutKneeFixed = 20.0, 0.5
kpRotorKneeFixed, kdRotorKneeFixed = getRotorGains(kpOutKneeFixed, kdOutKneeFixed)

kpOutKneeMoving, kdOutKneeMoving = 15.0, 2.0
kpRotorKneeMoving, kdRotorKneeMoving = getRotorGains(kpOutKneeMoving, kdOutKneeMoving)


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

def getOffset(motorID, serial, modelledInitialAngle):
    """Calibrate a motor and return its offset and initial raw angle."""
    cmd.motorType = MotorType.A1
    data.motorType = MotorType.A1
    cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
    cmd.id = motorID
    while not serial.sendRecv(cmd, data):
        print('\nWaiting for motor response...')

    rawInitialAngle = getOutputAngleDeg(data.q)
    offset = modelledInitialAngle - rawInitialAngle  # Offset calculation integrated here
    #time.sleep(0.002)  # 200 us
    return offset, rawInitialAngle

def calibrateJointReadings(serial):
    """Calibrate hip and knee motors and return offsets, initial angles, and calibration status."""
    hipOffset, hipAngleInitialRaw = getOffset(id.hip, serial, -90)
    kneeOffset, kneeAngleInitialRaw = getOffset(id.knee, serial, 0.0)


    # Check if the combined offset is within the acceptable range
    hipCalibration = 17.5 > hipAngleInitialRaw > 16.5
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


# Global variables for crouching state
crouchHeightMax = 0.33

hipAngleStart, hipAngleEnd, kneeAngleStart, kneeAngleEnd  = 0.0, 0.0, 0.0, 0.0
startCrouching, stopCrouching = False, True
crouchStartTime = 0.0

def getNewCrouchHeight(events,crouchHeightDesiredNew,crouchIncrement):
    global crouchHeightMax
    for event in events:
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                crouchHeightDesiredNew += crouchIncrement
            if event.key == pygame.K_DOWN:
                crouchHeightDesiredNew -= crouchIncrement

    crouchHeightDesiredNew = max(0.0 + crouchIncrement, min(crouchHeightMax, crouchHeightDesiredNew))
    return crouchHeightDesiredNew


def getLinearInterpolationAngle(startAngle, desiredAngle, T, t):
        currentAngle = (desiredAngle - startAngle) * t/T + startAngle
        #currentAngle = desiredAngle*t/T + startAngle*(1 - t/T)
        return currentAngle

def crouchControl(hipAngleCurrent, kneeAngleCurrent, heightDesiredPrev, heightDesiredNew, crouchDuration, crouching):

    global hipAngleStart, hipAngleEnd, kneeAngleStart, kneeAngleEnd
    global startCrouching, stopCrouching
    global crouchStartTime

    # Estimate current crouch height from FK
    xWheel, yWheel = forwardKinematicsDeg(hipAngleCurrent, kneeAngleCurrent)
    heightCurrent = abs(yWheel)

    startCrouching = (heightDesiredNew != heightDesiredPrev)

    if startCrouching and not crouching:
        hipAngleEnd, kneeAngleEnd = inverseKinematicsDeg(0.0, -heightDesiredNew, 'front')
        hipAngleStart, kneeAngleStart = hipAngleCurrent, kneeAngleCurrent
        hipAngleNew, kneeAngleNew = hipAngleStart, kneeAngleStart
        crouchStartTime = time.time()
        crouching = True
        stopCrouching = False
    elif crouching:
        dt = time.time() - crouchStartTime
        if dt >= crouchDuration:
            hipAngleNew, kneeAngleNew = hipAngleEnd, kneeAngleEnd
            crouching = False
            heightDesiredPrev = heightDesiredNew
        else:
            hipAngleNew = getLinearInterpolationAngle(hipAngleStart, hipAngleEnd, crouchDuration, dt)
            kneeAngleNew = getLinearInterpolationAngle(kneeAngleStart, kneeAngleEnd, crouchDuration, dt)
            print(f"\nAdjusting Crouch Height - Current: {heightCurrent:.3f}, Desired: {heightDesiredNew:.3f}")

    else:
        hipAngleNew, kneeAngleNew = hipAngleEnd, kneeAngleEnd
        heightDesiredPrev = heightDesiredNew
        print("\nCrouch Height Fixed\n")

    return hipAngleNew, kneeAngleNew, heightDesiredPrev, crouching


def chooseRotorGains(crouching):

    if crouching:
        # Return moving gains if crouching
        return kpRotorHipMoving, kdRotorHipMoving, kpRotorKneeMoving, kdRotorKneeMoving
    else:
        # Return fixed gains if not crouching
        return kpRotorHipFixed, kdRotorHipFixed, kpRotorKneeFixed, kdRotorKneeFixed


def plotAndSaveData(timeSteps,hipOutputAngles,kneeOutputAngles,hipCommandAngles,kneeCommandAngles, T, kp, kd):
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
    plt.legend(loc='best')
    plt.grid()

    # Base name for figure and CSV
    base_name = f"JointAngleOverTime_crouch_Time_{T:.1f}_kp_{kp:.1f}_kd_{kd:.1f}"

    # Save the figure
    figure_filename = f"{base_name}.png"
    plt.savefig(figure_filename, dpi=300)
    print(f"Figure saved as {figure_filename}")

    # Save the data as CSV
    csv_filename = f"{base_name}.csv"
    with open(csv_filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Time (s)', 'Hip Output Angle (deg)', 'Hip Command Angle (deg)','Knee Output Angle (deg)', 'Knee Command Angle (deg)'])
        for t, hip, knee in zip(timeSteps, hipOutputAngles, hipCommandAngles, kneeOutputAngles, kneeCommandAngles):
            writer.writerow([t, hipOut, hipCmd,  kneeOut, kneeCmd])

    print(f"Data saved as {csv_filename}")

    # Close the plot
    plt.close()

def find_f710():
    """Find the Logitech F710 controller device."""
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for device in devices:
        if "Logitech Gamepad F710" in device.name:
            print(f"Found Logitech F710 at {device.path}")
            return evdev.InputDevice(device.path)
    print("Logitech F710 not found. Ensure it is connected and in DirectInput mode.")
    exit()


#####<<<<<OLD CROUCH CODE>>>>####
'''
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
'''
