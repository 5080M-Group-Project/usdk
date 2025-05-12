
from functions import *

sys.path.append('../lib')
from unitree_actuator_sdk import *


##### NOTE 1: All rotor angles in RAD, all output angles in DEG########
##### NOTE 2: Whenever reading angles +offset, whenever commanding -offset. Offset in DEG######

#Initialize Loop Variables

# Serial Comms
rightLeg = leg.serial.right
leftLeg = leg.serial.left

# Data storage for plotting
leftHipOutputAngles, leftHipCommandAngles, leftHipOutputTorque, leftKneeOutputAngles, leftKneeCommandAngles, leftKneeOutputTorque, timeSteps = [], [], [], [], [], [], []
rightHipOutputAngles, rightHipCommandAngles, rightHipOutputTorque, rightKneeOutputAngles, rightKneeCommandAngles, rightKneeOutputTorque = [], [], [], [], [], []

hipTau, kneeTau = 0.0, 0.0

# Calibration Variables
offsetCalibration = False
leftHipOffset, leftKneeOffset = 0.0, 0.0
rightHipOffset, rightKneeOffset = 0.0, 0.0


# Crouching Initialisation
crouchingLeft, crouchingRight = False, False
crouchHeightDesiredPrevLeft, crouchHeightDesiredPrevRight = crouchHeightMax, crouchHeightMax
crouchHeightDesiredNewLeft, crouchHeightDesiredNewRight = 0.9*crouchHeightMax, 0.9*crouchHeightMax ### 0.9 or 0.75
crouchIncrement = 0.15*crouchHeightMax ### 0.1, 0.15, or 0.2
crouchDuration = 1.25

# LQR Initialisation
####################

try:
        while True:
                while not offsetCalibration:
                        ######<<<<<< CALIBRATION >>>>>>######
                        leftHipOffset, leftKneeOffset, leftHipOutputAngleDesired, leftKneeOutputAngleDesired, leftOffsetCalibration = calibrateJointReadings(leftLeg)
                        rightHipOffset, rightKneeOffset, rightHipOutputAngleDesired, rightKneeOutputAngleDesired, rightOffsetCalibration = calibrateJointReadings(rightLeg)
                        offsetCalibration = (leftOffsetCalibration and rightOffsetCalibration)
                        time.sleep(0.01)
                        leftHipOutputAngleCurrent, leftKneeOutputAngleCurrent = leftHipOutputAngleDesired, leftKneeOutputAngleDesired
                        rightHipOutputAngleCurrent, rightKneeOutputAngleCurrent = rightHipOutputAngleDesired, rightKneeOutputAngleDesired
                        if offsetCalibration:
                                globalStartTime = time.time()

                ######<<<<<< TIMING >>>>>>######
                loopStartTime = time.time()
                elapsedTime = loopStartTime - globalStartTime
                timeSteps.append(elapsedTime)


                ######<<<<<< GET INPUTS >>>>>>######
                #<<< LEFT LEG >>>#
                leftHipRotorAngleDesired, leftKneeRotorAngleDesired = getRotorAngleRad(leftHipOutputAngleDesired - leftHipOffset), getRotorAngleRad(leftKneeOutputAngleDesired - leftKneeOffset)
                kpRotorHipLeft, kdRotorHipLeft, kpRotorKneeLeft, kdRotorKneeLeft = chooseRotorGains(crouchingLeft)
                print(f'\n RAW INPUT - Left:  Hip: {leftHipRotorAngleDesired:.3f}, Knee: {leftKneeRotorAngleDesired:.3f}')

                # <<< RIGHT LEG >>>#
                rightHipRotorAngleDesired, rightKneeRotorAngleDesired = getRotorAngleRad(rightHipOutputAngleDesired - rightHipOffset), getRotorAngleRad(rightKneeOutputAngleDesired - rightKneeOffset)
                kpRotorHipRight, kdRotorHipRight, kpRotorKneeRight, kdRotorKneeRight = chooseRotorGains(crouchingRight)
                print(f'\n RAW INPUT - Right:  Hip: {rightHipRotorAngleDesired:.3f}, Knee: {rightKneeRotorAngleDesired:.3f}')

                # <<< WHEELS >>>#
                # Convert to rotor values, update gains


                ######<<<<<< COMMAND & DATA >>>>>>######
                #<<< LEFT HIP >>>#
                data = sendCmdRcvData(leftLeg, id.hip, kpRotorHipLeft, kdRotorHipLeft, leftHipRotorAngleDesired, 0.0, hipTau)
                leftHipOutputAngleCurrent = getOutputAngleDeg(data.q) + leftHipOffset
                hipTorque = calculateOutputTorque(kpRotorHipLeft, leftHipRotorAngleDesired, data.q, kdRotorHipLeft, 0.0, data.dq, hipTau)
                outputData(leftLeg, id.hip, data.q, leftHipOffset, data.dq, hipTorque, data.temp, data.merror,hipTau)
                leftHipOutputAngles.append(leftHipOutputAngleCurrent), leftHipCommandAngles.append(leftHipOutputAngleDesired), leftHipOutputTorque.append(hipTorque)

                # <<< RIGHT HIP >>>#
                data = sendCmdRcvData(rightLeg, id.hip, kpRotorHipRight, kdRotorHipRight, rightHipRotorAngleDesired,0.0, hipTau)
                rightHipOutputAngleCurrent = getOutputAngleDeg(data.q) + rightHipOffset
                hipTorque = calculateOutputTorque(kpRotorHipRight, rightHipRotorAngleDesired, data.q, kdRotorHipRight,0.0, data.dq, hipTau), outputData(rightLeg, id.hip, data.q,rightHipOffset, data.dq, hipTorque,data.temp, data.merror, hipTau)
                rightHipOutputAngles.append(rightHipOutputAngleCurrent), rightHipCommandAngles.append(rightHipOutputAngleDesired), rightHipOutputTorque.append(hipTorque)

                #<<< LEFT KNEE >>>#
                data = sendCmdRcvData(leftLeg, id.knee, kpRotorKneeLeft, kdRotorKneeLeft, leftKneeRotorAngleDesired,0.0, kneeTau)
                leftKneeOutputAngleCurrent = getOutputAngleDeg(data.q) + leftKneeOffset
                kneeTorque = calculateOutputTorque(kpRotorKneeLeft, leftKneeRotorAngleDesired, data.q, kdRotorKneeLeft, 0.0, data.dq, kneeTau)
                outputData(leftLeg, id.knee, data.q, leftKneeOffset, data.dq, kneeTorque, data.temp, data.merror, kneeTau)
                leftKneeOutputAngles.append(leftKneeOutputAngleCurrent), leftKneeCommandAngles.append(leftKneeOutputAngleDesired), leftKneeOutputTorque.append(kneeTorque)

                # <<< RIGHT KNEE >>>#
                data = sendCmdRcvData(rightLeg, id.knee, kpRotorKneeRight, kdRotorKneeRight, rightKneeRotorAngleDesired,0.0, kneeTau)
                rightKneeOutputAngleCurrent = getOutputAngleDeg(data.q) + rightKneeOffset
                kneeTorque = calculateOutputTorque(kpRotorKneeRight, rightKneeRotorAngleDesired, data.q,kdRotorKneeRight, 0.0, data.dq, kneeTau)
                outputData(rightLeg, id.knee, data.q, rightKneeOffset, data.dq, kneeTorque, data.temp, data.merror,kneeTau)
                rightKneeOutputAngles.append(rightKneeOutputAngleCurrent), rightKneeCommandAngles.append(rightKneeOutputAngleDesired), rightKneeOutputTorque.append(kneeTorque)

                #<<< LEFT WHEEL >>>#
                #Send wheel commands, estimate torque, log data

                #<<< RIGHT WHEEL >>>#
                #Send wheel commands, estimate torque, log data

                ######<<<<<< CROUCHING >>>>>>######
                #<<< LEFT LEG >>>#
                crouchHeightDesiredNewLeft = getCrouchCommand(pygame.event.get(), crouchHeightDesiredNewLeft, crouchIncrement)
                leftHipOutputAngleDesired, leftKneeOutputAngleDesired, crouchHeightDesiredPrevLeft, crouchingLeft \
                        = crouchControl(leftLeg, leftHipOutputAngleCurrent, leftKneeOutputAngleCurrent, crouchHeightDesiredPrevLeft, crouchHeightDesiredNewLeft, crouchDuration, crouchingLeft)

                #<<< RIGHT LEG >>>#
                crouchHeightDesiredNewRight = getCrouchCommand(pygame.event.get(), crouchHeightDesiredNewRight, crouchIncrement)
                rightHipOutputAngleDesired, rightKneeOutputAngleDesired, crouchHeightDesiredPrevRight, crouchingRight \
                        = crouchControl(rightLeg, rightHipOutputAngleCurrent, rightKneeOutputAngleCurrent, crouchHeightDesiredPrevRight, crouchHeightDesiredNewRight, crouchDuration, crouchingRight)

                ######<<<<<< BALANCE >>>>>>######
                #In-Loop LQR Control Input Generation

                ######<<<<<< TIMING >>>>>>######
                loopTime = time.time() - loopStartTime
                print(f"Loop Time: {loopTime}\n")

except KeyboardInterrupt:

        print("\nLoop stopped by user. Saving figure...")
        try:
                plotAndSaveLegData(leftLeg, timeSteps, leftHipOutputAngles, leftHipCommandAngles, leftHipOutputTorque, leftKneeOutputAngles, leftKneeCommandAngles, leftKneeOutputTorque, crouchDuration)
                plotAndSaveLegData(rightLeg, timeSteps, rightHipOutputAngles, rightHipCommandAngles, rightHipOutputTorque, rightKneeOutputAngles, rightKneeCommandAngles, rightKneeOutputTorque, crouchDuration)
                #plotAndSaveBalanceData(timeSteps, wOutLeftLog, wCmdLeftLog, tLeftLog,wOutRightLog, wCmdRightLog, tRightLog, pitchLog, pitchRateLog, yawLog, yawRateLog)
                print(f"Error encountered while saving figure: {e}")
        finally:
        sys.exit(0)  # Ensure clean exit
