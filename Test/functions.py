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

pygame.init()
screen = pygame.display.set_mode((400, 300)) # NEEDED????

cmd.motorType = MotorType.A1
data.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)

gearRatio = queryGearRatio(MotorType.A1)

# Global variables for crouching state
crouchHeightMax = 0.33
hipAngleStart, hipAngleEnd, kneeAngleStart, kneeAngleEnd  = 0.0, 0.0, 0.0, 0.0
startCrouching, stopCrouching = False, True
crouchStartTime = 0.0

hipCommsSuccess, hipCommsFail, kneeCommsSuccess, kneeCommsFail, wheelCommsSucces, wheelsCommsFail = 0, 0, 0, 0, 0, 0

#### Class for serial? ####
class leg:
    @staticmethod
    def getName(serialPort):
        if serialPort == leftLeg:
            return 'Left'
        if serialPort == rightLeg:
            return 'Right'


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

    @staticmethod
    def logCommsFail(motorID):
        # Return motor name based on the motor ID value
        global hipCommsFail, kneeCommsFail, wheelCommsFail
        if motorID == id.hip:
            hipCommsFail += 1
            return hipCommsFail
        elif motorID == id.knee:
            kneeCommsFail += 1
            return kneeCommsFail
        elif motorID == id.wheel:
            wheelCommsFail += 1
            return wheelCommsFail
        else:
            return None

# Function to convert output gains to rotor gains
def getRotorGains(kpOutput, kdOutput):
    kpRotor = (kpOutput / (gearRatio * gearRatio)) / 26.07
    kdRotor = (kdOutput / (gearRatio * gearRatio)) * 100.0
    return kpRotor, kdRotor

kpOutputMoving, kdOutputMoving = 15.0, 2.0
kpOutputFixed, kdOutputFixed = 20.0, 0.8

class rotorGains:

    global kpOutputMoving, kdOutputMoving, kpOutputFixed, kdOutputFixed

    class hip:
        # Separate KP and KD for hip
        class fixed:
            kp, kd = getRotorGains(kpOutputFixed, kdOutputFixed)  # Fixed values for hip

        class moving:
            kp, kd = getRotorGains(kpOutputMoving, kdOutputMoving)  # Moving values for hip
    class knee:
        # Separate KP and KD for knee
        class fixed:
            kp, kd = getRotorGains(kpOutputFixed, kdOutputFixed)  # Fixed values for knee

        class moving:
            kp, kd = getRotorGains(kpOutputMoving, kdOutputMoving)  # Moving values for knee

# Function to get the current motor output angle in DEGREES from rotor in RAD
def getOutputAngleDeg(rotorAngle):
    return (rotorAngle / gearRatio) * (180 / np.pi)

# Function to compute the desired rotor angle in RADIANS from output in RAD
def getRotorAngleRad(outputAngle):
    return float (outputAngle * (np.pi / 180)) * gearRatio

def sendCmdRcvData(serialPort, ID, kp, kd, q, dq, tau):
    cmd.id = ID
    cmd.kp = kp  # proportional or position term. i.e. stiffness
    cmd.kd = kd  # derivative or velocity term, i.e damping
    cmd.q = q    # angle, radians
    cmd.dq = dq  # angular velocity, radians/s
    cmd.tau = tau  # rotor feedforward torque

    #NEEDED?
    while not serialPort.sendRecv(cmd, data):
        commsFail = id.logCommsFail(ID)
        print(f'Waiting for {leg.getName(serial)} {id.getName(ID)} motor to respond. Response lost {commsFail} times')
    return data

# Function to compute output torque
def calculateOutputTorque(kp, qDesired, qCurrent, kd, dqDesired, dqCurrent, tau):
    outputTorque = tau + kp * (qDesired - qCurrent) + kd * (dqDesired - dqCurrent)
    if outputTorque > 30:
        print(f'[WARNING] Commands result in excessive Toqrue ({outputTorque} Nm) asked of the actuator ')
    return outputTorque

# Function to output motor data
def outputData(serial, motorID, qRotorRads, offset, dqRotorRads, torqueNm, temperature, motorError,tau):
        legLabel = leg.getName(serial)
        motorLabel = id.getName(motorID)

        print("\n")
        print(f"<<<<<{legLabel.upper()} {motorLabel.upper()} MOTOR>>>>>")
        print(f"Angle (Deg): {getOutputAngleDeg(qRotorRads) + offset}")
        print(f"Angular Velocity (rad/s): {dqRotorRads / gearRatio}")
        print(f"Torque (N.m): {torqueNm}")
        print(f"Tau (N.m): {tau}")
        print(f"Temperature: {temperature}")
        print(f"ISSUE? {motorError}")
        print("\n")


def getOffset(serialPort, motorID, modelledInitialAngle, kp, kd, fix):
    """Calibrate a motor and return its offset and initial raw angle."""
    kpRotor, kdRotor = getRotorGains(kp, kd)
    if fix is None:
        cmd.id = motorID
        cmd.dq = 0.0
        cmd.kp = kpRotor
        cmd.kd = kdRotor
    else:
        fixRotor = getRotorAngleRad(fix)
        cmd.id = motorID
        cmd.q = fixRotor
        cmd.dq = 0.0
        cmd.kp = kpRotor
        cmd.kd = kdRotor

    while not serialPort.sendRecv(cmd, data):
        print(f'\nWaiting for {leg.getName(serialPort)} {id.getName(motorID)} motor response...\n')

    rawInitialAngle = getOutputAngleDeg(data.q)
    offset = modelledInitialAngle - rawInitialAngle  # Offset calculation integrated here
    return offset, rawInitialAngle

kpHipCalibration, kpKneeCalibration = 0.0, 0.0
kdHipCalibration, kdKneeCalibration = 0.25, 0.25
hipCalibrationFix, kneeCalibrationFix = None, None

def calibrateJointReadings(serialPort):
    """Calibrate hip and knee motors and return offsets, initial angles, and calibration status."""
    global kpHipCalibration, kpKneeCalibration, kdHipCalibration, kdKneeCalibration, hipCalibrationFix, kneeCalibrationFix

    hipOffset, hipAngleInitialRaw = getOffset(serialPort, id.hip, -90, kpHipCalibration, kdHipCalibration, hipCalibrationFix)
    kneeOffset, kneeAngleInitialRaw = getOffset(serialPort, id.knee, 0.0, kpKneeCalibration, kdKneeCalibration, kneeCalibrationFix)

    # Check if the combined offset is within the acceptable range
    hipCalibration = (24.5 < hipAngleInitialRaw < 25.5) or (38.5 < hipAngleInitialRaw < 39.5) or (-0.5 < hipAngleInitialRaw < 0.5) #leg1 leg2 leg2
    if hipCalibration:
        kpHipCalibration = 30.0
        hipCalibrationFix = hipAngleInitialRaw
        print(f"\n {leg.getName(serialPort)} Hip Locked\n")

    kneeCalibration = (-0.5 < kneeAngleInitialRaw < 0.5) or (39.5 < kneeAngleInitialRaw < 40.5)  or (12.0 < kneeAngleInitialRaw < 13.0) #leg1 leg1 leg2
    if kneeCalibration:
        kpKneeCalibration = 30.0
        kneeCalibrationFix = kneeAngleInitialRaw
        print(f"\n {leg.getName(serialPort)} Knee Locked\n")

    offsetCalibration = hipCalibration and kneeCalibration
    if offsetCalibration:
        print(f"\n {leg.getName(serialPort)} Leg Angle Offsets Calibrated - Hip: {hipOffset:.6f}, Knee: {kneeOffset:.6f}\n")
        hipOutputAngleDesired, kneeOutputAngleDesired = hipAngleInitialRaw + hipOffset, kneeAngleInitialRaw + kneeOffset
        # Return offsets, desired angles, and calibration status
        return hipOffset, kneeOffset, hipOutputAngleDesired, kneeOutputAngleDesired, True

    # Return offsets and status when calibration is not successful
    print(f"\nRaw Initial Angles ({leg.getName(serial)} Leg) - Hip: {hipAngleInitialRaw:.6f}, Knee: {kneeAngleInitialRaw:.6f}\n")
    return hipOffset, kneeOffset, None, None, False

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

def getCrouchCommand(events,crouchHeightDesiredNew,crouchIncrement):
    global crouchHeightMax
    crouchHeightMin = 0.3*crouchHeightMax
    for event in events:
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                crouchHeightDesiredNew += crouchIncrement
            if event.key == pygame.K_DOWN:
                crouchHeightDesiredNew -= crouchIncrement

    crouchHeightDesiredNew = max(crouchHeightMin, min(0.99999*crouchHeightMax, crouchHeightDesiredNew))
    return crouchHeightDesiredNew

def getLinearInterpolationAngle(startAngle, desiredAngle, T, t):
        currentAngle = (desiredAngle - startAngle) * t/T + startAngle
        #currentAngle = desiredAngle*t/T + startAngle*(1 - t/T)
        return currentAngle

def crouchControl(kneeDirection ,hipAngleCurrent, kneeAngleCurrent, heightDesiredPrev, heightDesiredNew, crouchDuration, crouching):

    global hipAngleStart, hipAngleEnd, kneeAngleStart, kneeAngleEnd
    global startCrouching, stopCrouching
    global crouchStartTime

    # Estimate current crouch height from FK
    xWheel, yWheel = forwardKinematicsDeg(hipAngleCurrent, kneeAngleCurrent)
    heightCurrent = abs(yWheel)

    startCrouching = (heightDesiredNew != heightDesiredPrev)

    if startCrouching and not crouching:
        hipAngleEnd, kneeAngleEnd = inverseKinematicsDeg(0.0, -heightDesiredNew, kneeDirection)
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
    rotor = rotorGains()
    if crouching:
        # Return moving gains if crouching
        return rotor.hip.moving.kp, rotor.hip.moving.kd, rotor.knee.moving.kp, rotor.knee.moving.kd
    else:
        # Return fixed gains if not crouching
        return rotor.hip.fixed.kp, rotor.hip.fixed.kd, rotor.knee.fixed.kp, rotor.knee.fixed.kd

def plotAndSaveLegData(serial, timeSteps, hipOutputAngles, hipCommandAngles, hipTorque, kneeOutputAngles, kneeCommandAngles, kneeTorque, T):
    # Ensure all lists have the same length
    global kpOutputMoving, kdOutputMoving, kpOutputFixed, kdOutputFixed

    min_length = min(len(timeSteps), len(hipOutputAngles), len(kneeOutputAngles), len(hipCommandAngles),
                     len(kneeCommandAngles), len(hipTorque), len(kneeTorque))

    if min_length == 0:
        print("No data collected. Exiting without saving.")
        sys.exit(0)  # Exit safely if no data

    timeSteps = timeSteps[:min_length]
    hipOutputAngles = hipOutputAngles[:min_length]
    kneeOutputAngles = kneeOutputAngles[:min_length]
    hipCommandAngles = hipCommandAngles[:min_length]
    kneeCommandAngles = kneeCommandAngles[:min_length]
    hipTorque = hipTorque[:min_length]
    kneeTorque = kneeTorque[:min_length]


    # Plotting
    plt.figure()
    plt.plot(timeSteps, hipOutputAngles, label='Hip Output Angles')
    plt.plot(timeSteps, kneeOutputAngles, label='Knee Output Angles')
    plt.plot(timeSteps, hipCommandAngles, label='Hip Command Angle')
    plt.plot(timeSteps, kneeCommandAngles, label='Knee Command Angle')

    ##plot on seperate y-axis?
    plt.plot(timeSteps, hipTorque, label='Hip Output Torque')
    plt.plot(timeSteps, kneeTorque, label='Knee Output Torque')

    plt.xlabel('Time (s)')
    plt.ylabel('Angle (deg)')
    plt.title(f'{leg.getName(serial)} Hip and Knee Angles Over Time')
    plt.legend(loc='best')
    plt.grid()

    # Base name for figure and CSV
    base_name = f"{leg.getName(serial)} JointAngleOverTime_crouch_Time_{T:.1f}_moving_kp_{kpOutputMoving:.1f}_kd_{kdOutputMoving:.1f}_fixed_kp_{kpOutputFixed:.1f}_kd_{kdOutputFixed:.1f}"

    # Save the figure
    figure_filename = f"{base_name}.png"
    plt.savefig(figure_filename, dpi=300)
    print(f"Figure saved as {figure_filename}")

    # Save the data as CSV
    csv_filename = f"{base_name}.csv"
    with open(csv_filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Time (s)', 'Hip Output Angle (deg)', 'Hip Command Angle (deg)','Hip Output Torque (Nm)','Knee Output Angle (deg)', 'Knee Command Angle (deg)', 'Knee Output Torque (Nm)'])
        for t, hipOut, hipCmd,hipT, kneeOut, kneeCmd, kneeT in zip(timeSteps, hipOutputAngles, hipCommandAngles, hipTorque, kneeOutputAngles, kneeCommandAngles, kneeTorque):
            writer.writerow([t, hipOut, hipCmd,hipT, kneeOut, kneeCmd, kneeT])

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
count = 0
thetaHipVector = []
thetaKneeVector = []

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
