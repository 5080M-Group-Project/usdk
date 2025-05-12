import time
import sys
import os
import csv
import numpy as np
import matplotlib.pyplot as plt
import pygame

sys.path.append('../lib')
from unitree_actuator_sdk import *

cmd = MotorCmd()
data = MotorData()

pygame.init()
screen = pygame.display.set_mode((400, 300)) #

cmd.motorType = MotorType.A1
data.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)

gearRatio = queryGearRatio(MotorType.A1) #9.1 for A1 Motor

#Link lengths
L1 = 0.165  # Length of link 1
L2 = 0.165  # Length of link 2

# Global variables for crouching state
crouchHeightMax = 0.33
startCrouchingLeft, stopCrouchingLeft = False, True
leftHipAngleStart, leftHipAngleEnd, leftKneeAngleStart, leftKneeAngleEnd = 0.0, 0.0, 0.0, 0.0
startCrouchingRight, stopCrouchingRight = False, True
rightHipAngleStart, rightHipAngleEnd, rightKneeAngleStart, rightKneeAngleEnd  = 0.0, 0.0, 0.0, 0.0
crouchStartTime = 0.0

hipCommsSuccess, hipCommsFail, kneeCommsSuccess, kneeCommsFail, wheelCommsSucces, wheelsCommsFail = 0, 0, 0, 0, 0, 0

class leg:
    class serial:
        right = SerialPort('/dev/ttyUSB0')
        left = SerialPort('/dev/ttyUSB1')

    @staticmethod
    def getName(serial):
        if serial == leg.serial.left:
            return 'Left'
        elif serial == leg.serial.right:
            return 'Right'
        else:
            return 'Unknown'

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
kpOutputFixed, kdOutputFixed = 30, 0.8

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

    while not serialPort.sendRecv(cmd, data):
        commsFail = id.logCommsFail(ID)
        print(f'Waiting for {leg.getName(serialPort)} {id.getName(ID)} motor to respond. Response lost {commsFail} times')
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

kpCalibration, kpFixed = 0.0, 30.0
kdCalibration = 0.25

leftHipCalibrationFix, rightHipCalibrationFix, leftKneeCalibrationFix, rightKneeCalibrationFix = None, None, None, None
kpLeftHipCalibration, kpLeftKneeCalibration, kpRightHipCalibration, kpRightKneeCalibration = 0.0, 0.0, 0.0, 0.0

def calibrateJointReadings(serialPort):
    """Calibrate hip and knee motors and return offsets, initial angles, and calibration status."""
    global kpCalibration, kdCalibration, kpFixed
    global kpLeftHipCalibration, kpLeftKneeCalibration, kpRightHipCalibration, kpRightKneeCalibration
    global leftHipCalibrationFix, leftKneeCalibrationFix, rightHipCalibrationFix, rightKneeCalibrationFix

    if leg.getName(serialPort) == 'Left':
        hipOffset, hipAngleInitialRaw = getOffset(serialPort, id.hip, -90, kpLeftHipCalibration, kdCalibration, leftHipCalibrationFix)
        kneeOffset, kneeAngleInitialRaw = getOffset(serialPort, id.knee, 0.0, kpLeftKneeCalibration, kdCalibration, leftKneeCalibrationFix)

        hipCalibration =  (33.5 < hipAngleInitialRaw < 34.5)  #or (24.5 < hipAngleInitialRaw < 25.5)
        if hipCalibration:
            kpLeftHipCalibration = kpFixed
            leftHipCalibrationFix = hipAngleInitialRaw
            print(f"\n {leg.getName(serialPort)} Hip Locked\n")

        kneeCalibration =  (-0.5 < kneeAngleInitialRaw < 0.5) or (39.5 < kneeAngleInitialRaw < 40.5)
        if kneeCalibration:
            kpLeftKneeCalibration = 30.0
            leftKneeCalibrationFix = kneeAngleInitialRaw
            print(f"\n {leg.getName(serialPort)} Knee Locked\n")

    elif leg.getName(serialPort) == 'Right':
        hipOffset, hipAngleInitialRaw = getOffset(serialPort, id.hip, -90, kpRightHipCalibration, kdCalibration,rightHipCalibrationFix)
        kneeOffset, kneeAngleInitialRaw = getOffset(serialPort, id.knee, 0.0, kpRightKneeCalibration, kdCalibration, rightKneeCalibrationFix)

        hipCalibration = (-0.5 < hipAngleInitialRaw < 0.5) or (35.5 < hipAngleInitialRaw < 36.5)
        if hipCalibration:
            kpRightHipCalibration = 30.0
            rightHipCalibrationFix = hipAngleInitialRaw
            print(f"\n {leg.getName(serialPort)} Hip Locked\n")

        kneeCalibration = (12.0 < kneeAngleInitialRaw < 13.0)
        if kneeCalibration:
            kpRightKneeCalibration = 30.0
            rightKneeCalibrationFix = kneeAngleInitialRaw
            print(f"\n {leg.getName(serialPort)} Knee Locked\n")

    # Check if the combined offset is within the acceptable range
    offsetCalibration = hipCalibration and kneeCalibration
    if offsetCalibration:
        print(f"\n {leg.getName(serialPort)} Leg Angle Offsets Calibrated - Hip: {hipOffset:.6f}, Knee: {kneeOffset:.6f}\n")
        hipOutputAngleDesired, kneeOutputAngleDesired = hipAngleInitialRaw + hipOffset, kneeAngleInitialRaw + kneeOffset
        # Return offsets, desired angles, and calibration status
        return hipOffset, kneeOffset, hipOutputAngleDesired, kneeOutputAngleDesired, True

    # Return offsets and status when calibration is not successful
    print(f"\nRaw Initial Angles ({leg.getName(serialPort)} Leg) - Hip: {hipAngleInitialRaw:.6f}, Knee: {kneeAngleInitialRaw:.6f}\n")
    return hipOffset, kneeOffset, None, None, False

def forwardKinematicsDeg(thetaHip, thetaKnee, frame):
    L1 = 0.165  # Length of link 1
    L2 = 0.165  # Length of link 2

    xKnee = L1*np.cos(thetaHip)
    yKnee = L1*np.sin(thetaHip)

    yWheel = yKnee + L2*np.sin(thetaKnee + thetaHip)
    xWheel = xKnee + L2*np.cos(thetaKnee + thetaHip)

    if frame.lower() == "wheel":
        return xWheel, yWheel
    elif frame.lower() == "knee":
        return xKnee, yKnee

def inverseKinematicsDeg(xdes, ydes, kneeDir):
    # Define link lengths
    global L1, L2

    C = np.sqrt(xdes ** 2 + ydes ** 2)
    if C > L1 + L2:
        print("\nOut of range!\n")
        return None, None

    # Calculate intermediate angles
    beta = np.arccos((L1**2 + L2**2 - xdes**2 - ydes**2) / (2 * L1 * L2))
    alpha = np.arccos((xdes**2 + ydes**2 + L1**2 - L2**2) / (2 * L1 * C))
    gamma = np.arctan2(ydes, xdes)

    # Handle kneeDir input
    kneeDir = kneeDir.lower()
    if kneeDir not in ['front', 'back']:
        print("\nInvalid Knee Direction!\n")
        return None, None

    if kneeDir == 'front':
        thetaHip = (gamma - alpha) * (180.0 / np.pi)
        thetaKnee = (np.pi - beta) * (180.0 / np.pi)
        return thetaHip, thetaKnee

    elif kneeDir == 'back':
        thetaHip = (gamma + alpha) * (180.0 / np.pi)
        thetaKnee = (beta - np.pi) * (180.0 / np.pi)
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
    #crouchHeightDesiredNew = np.clip(crouchHeightDesiredNew, crouchHeightMin, crouchHeightMax*0.99999)
    return crouchHeightDesiredNew

def getLinearInterpolationAngle(startAngle, desiredAngle, T, t):
        currentAngle = (desiredAngle - startAngle) * t/T + startAngle
        #currentAngle = desiredAngle*t/T + startAngle*(1 - t/T)
        return currentAngle

def crouchControl(serial ,hipAngleCurrent, kneeAngleCurrent, heightDesiredPrev, heightDesiredNew, crouchDuration, crouching):

    global leftHipAngleStart, leftHipAngleEnd, leftKneeAngleStart, leftKneeAngleEnd
    global rightHipAngleStart, rightHipAngleEnd, rightKneeAngleStart, rightKneeAngleEnd
    global startCrouching, stopCrouching, crouchStartTime

    # Estimate current crouch height from FK
    xWheel, yWheel = forwardKinematicsDeg(hipAngleCurrent, kneeAngleCurrent, "wheel")
    heightCurrent = abs(yWheel)

    startCrouching = (heightDesiredNew != heightDesiredPrev)

    if leg.getName(serial) == 'Left':
        if startCrouching and not crouching:
            leftHipAngleEnd, leftKneeAngleEnd = inverseKinematicsDeg(0.0, -heightDesiredNew, 'back')
            leftHipAngleStart, leftKneeAngleStart = hipAngleCurrent, kneeAngleCurrent
            hipAngleNew, kneeAngleNew = leftHipAngleStart, leftKneeAngleStart
            crouchStartTime = time.time()
            crouching = True
            stopCrouching = False
        elif crouching:
            dt = time.time() - crouchStartTime
            if dt >= crouchDuration:
                hipAngleNew, kneeAngleNew = leftHipAngleEnd, leftKneeAngleEnd
                crouching = False
                heightDesiredPrev = heightDesiredNew
            else:
                hipAngleNew = getLinearInterpolationAngle(leftHipAngleStart, leftHipAngleEnd, crouchDuration, dt)
                kneeAngleNew = getLinearInterpolationAngle(leftKneeAngleStart, leftKneeAngleEnd, crouchDuration, dt)
                print(f"\nAdjusting Crouch Height - Current: {heightCurrent:.3f}, Desired: {heightDesiredNew:.3f}")
        else:
            hipAngleNew, kneeAngleNew = leftHipAngleEnd, leftKneeAngleEnd
            heightDesiredPrev = heightDesiredNew
            print("\nCrouch Height Fixed\n")

    elif leg.getName(serial) == 'Right':
        if startCrouching and not crouching:
            rightHipAngleEnd, rightKneeAngleEnd = inverseKinematicsDeg(0.0, -heightDesiredNew, 'front')
            rightHipAngleStart, rightKneeAngleStart = hipAngleCurrent, kneeAngleCurrent
            hipAngleNew, kneeAngleNew = rightHipAngleStart, rightKneeAngleStart
            crouchStartTime = time.time()
            crouching = True
            stopCrouching = False
        elif crouching:
            dt = time.time() - crouchStartTime
            if dt >= crouchDuration:
                hipAngleNew, kneeAngleNew = rightHipAngleEnd, rightKneeAngleEnd
                crouching = False
                heightDesiredPrev = heightDesiredNew
            else:
                hipAngleNew = getLinearInterpolationAngle(rightHipAngleStart, rightHipAngleEnd, crouchDuration, dt)
                kneeAngleNew = getLinearInterpolationAngle(rightKneeAngleStart, rightKneeAngleEnd, crouchDuration, dt)
                print(f"\nAdjusting Crouch Height - Current: {heightCurrent:.3f}, Desired: {heightDesiredNew:.3f}")

        else:
            hipAngleNew, kneeAngleNew = rightHipAngleEnd, rightKneeAngleEnd
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


# masses (kg)
actuatorMass     = 0.5        # same motors
L1Mass        = 1.2
L2Mass        = 1.0
connectorMass    = 0.1        # mass of the wheel–leg connector
wheelBodyMass    = 0.3        # just the wheel
mainBodyMass     = 8.0        # robot’s body mass

# COM offset fractions along each link
L1ComOffset = 0.5 #GET
L2ComOffset = 0.5 #GET

# wheel assembly mass
wheelMass = actuatorMass + connectorMass + wheelBodyMass

def computeLegCom(qHip, qKnee):
    global L1, L2, L1Mass, L2Mass, L1ComOffset, L2ComOffset, wheelMass, actuatorMass
    """
    Compute the (x,y) CoM of one planar 2R leg+wheel,
    measured from hip.
    """
    # hip at origin
    rHip = np.array([0.0, 0.0])

    # get knee position via updated FK
    xKnee, yKnee = forwardKinematicsDeg(qHip, qKnee, "knee")
    rKnee = np.array([xKnee, yKnee])

    # get wheel position via updated FK
    xWheel, yWheel = forwardKinematicsDeg(qHip, qKnee, "wheel")
    rWheel = np.array([xWheel, yWheel])

    # link-1 COM at fraction along thigh
    L1com = L1ComOffset * rKnee

    # link-2 COM at fraction along shank
    L2com = rKnee + L2ComOffset * (rWheel - rKnee)

    # mass-weighted sum of all point masses
    S = (actuatorMass * rHip
      + L1Mass    * L1com
      + actuatorMass * rKnee
      + L2Mass    * L2com
      + wheelMass    * rWheel)

    Mtot = actuatorMass + L1Mass + actuatorMass + L2Mass + wheelMass

    com2D = S / Mtot
    return float(com2D[0]), float(com2D[1])

def computeRobotCom(qHipLeft, qKneeLeft, qHipRight, qKneeRight):
    global L1, L2, wheelMass, actuatorMass, mainBodyMass
    # lateral distance between the two hip joints (m)
    legSeparation = 0.4  # CHECK
    """
    Compute the full 3-D CoM [x, y, z] of:
      – left leg+wheel (in its sagittal plane at +z/2)
      – right leg+wheel (at -z/2)
      – main body (assumed centered at origin)
    Angles in radians. Returns metres.
    """
    # each leg's sagittal CoM
    xL, yL = computeLegCom(qHipLeft,  qKneeLeft)
    xR, yR = computeLegCom(qHipRight, qKneeRight)

    # lift into 3D with ±z offsets
    halfSep = legSeparation / 2.0
    rLeft  = np.array([xL, yL, +halfSep])
    rRight = np.array([xR, yR, -halfSep])

    # main body COM at origin
    rBody = np.array([0.0, 0.0, 0.0])

    # masses
    Mleg  = actuatorMass + link1Mass + actuatorMass + link2Mass + wheelMass
    Mbody = mainBodyMass

    # weighted sum & normalize
    S    = Mbody * rBody + Mleg * rLeft + Mleg * rRight
    Mtot = Mbody + 2.0 * Mleg
    return S / Mtot

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


def plotAndSaveBalanceData(timeSteps, leftWheelOutputW, leftWheelCommandW, leftWheelOutputT, rightWheelOutputW, rightWheelCommandW, rightWheelOutput, pitch, pitchRate, yaw, yawRate):
    # Ensure all lists have the same length
    min_length = min(len(timeSteps), len(leftWheelOutputW), len(leftWheelCommandW), len(leftWheelOutputT),
                     len(rightWheelOutputW), len(rightWheelCommandW), len(rightWheelOutput), len(pitch), len(pitchRate), len(yaw), len(yawRate))

    if min_length == 0:
        print("No data collected. Exiting without saving.")
        sys.exit(0)  # Exit safely if no data

    # Truncate lists to the minimum length
    timeSteps = timeSteps[:min_length]
    leftWheelOutputW = leftWheelOutputW[:min_length]
    leftWheelCommandW = leftWheelCommandW[:min_length]
    leftWheelOutputT = leftWheelOutputT[:min_length]
    rightWheelOutputW = rightWheelOutputW[:min_length]
    rightWheelCommandW = rightWheelCommandW[:min_length]
    rightWheelOutput = rightWheelOutput[:min_length]
    pitch = pitch[:min_length]
    pitchRate = pitchRate[:min_length]
    yaw = yaw[:min_length]
    yawRate = yawRate[:min_length]

    # Plotting
    plt.figure()

    # Plot wheel outputs and commands
    plt.plot(timeSteps, leftWheelOutputW, label='Left Wheel Output (W)')
    plt.plot(timeSteps, leftWheelCommandW, label='Left Wheel Command (W)')
    plt.plot(timeSteps, leftWheelOutputT, label='Left Wheel Output Torque (Nm)')
    plt.plot(timeSteps, rightWheelOutputW, label='Right Wheel Output (W)')
    plt.plot(timeSteps, rightWheelCommandW, label='Right Wheel Command (W)')
    plt.plot(timeSteps, rightWheelOutput, label='Right Wheel Output Torque (Nm)')

    # Plot pitch and yaw data
    plt.plot(timeSteps, pitch, label='Pitch (deg)')
    plt.plot(timeSteps, pitchRate, label='Pitch Rate (deg/s)')
    plt.plot(timeSteps, yaw, label='Yaw (deg)')
    plt.plot(timeSteps, yawRate, label='Yaw Rate (deg/s)')

    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('Wheeled Biped Balance Data Over Time')
    plt.legend(loc='best')
    plt.grid()

    # Base name for figure and CSV
    base_name = f"BalanceData_Time_{timeSteps[0]:.1f}_to_{timeSteps[-1]:.1f}"

    # Save the figure
    figure_filename = f"{base_name}.png"
    plt.savefig(figure_filename, dpi=300)
    print(f"Figure saved as {figure_filename}")

    # Save the data as CSV
    csv_filename = f"{base_name}.csv"
    with open(csv_filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Time (s)', 'Left Wheel Output (W)', 'Left Wheel Command (W)', 'Left Wheel Output Torque (Nm)',
                         'Right Wheel Output (W)', 'Right Wheel Command (W)', 'Right Wheel Output Torque (Nm)',
                         'Pitch (deg)', 'Pitch Rate (deg/s)', 'Yaw (deg)', 'Yaw Rate (deg/s)'])
        for t, lwOutW, lwCmdW, lwOutT, rwOutW, rwCmdW, rwOut, p, pRate, y, yRate in zip(timeSteps, leftWheelOutputW, leftWheelCommandW, leftWheelOutputT,
                                                                                    rightWheelOutputW, rightWheelCommandW, rightWheelOutput, pitch, pitchRate, yaw, yawRate):
            writer.writerow([t, lwOutW, lwCmdW, lwOutT, rwOutW, rwCmdW, rwOut, p, pRate, y, yRate])

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
