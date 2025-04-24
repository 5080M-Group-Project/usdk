
from functions import *

######## NEEDED??? ########
sys.path.append('../lib')
from unitree_actuator_sdk import *
############################

rightLeg = leg.serial.right
leftLeg = leg.serial.left

##### NOTE 1: All rotor angles in RAD, all output angles in DEG########
##### NOTE 2: Whenever reading angles +offset, whenever commanding -offset. Offset in DEG######

#Initialize Loop Variables

# Data storage for plotting
leftHipOutputAngles, leftHipCommandAngles, leftHipOutputTorque, leftKneeOutputAngles, leftKneeCommandAngles, leftKneeOutputTorque, timeSteps = [], [], [], [], [], [], []
rightHipOutputAngles, rightHipCommandAngles, rightHipOutputTorque, rightKneeOutputAngles, rightKneeCommandAngles, rightKneeOutputTorque = [], [], [], [], [], []


#NEEDED?
leftHipOffset, leftKneeOffset = 0.0, 0.0
rightHipOffset, rightKneeOffset = 0.0, 0.0
hipTau, kneeTau = 0.0, 0.0

offsetCalibration = False

#Crouching Initialisation
crouching = False
crouchHeightDesiredPrev = crouchHeightMax
crouchHeightDesiredNew = 0.9*crouchHeightMax ### 0.9 or 0.75
crouchIncrement = 0.15*crouchHeightMax ### 0.1, 0.15, or 0.2
crouchDuration = 1.00 ### relative to the above ^, ~0.75s, ~1.25s???, ?????  #### MOVE TO FUNCTIONS, MAKE GLOBAL WHERE NEEDED

try:
        while True:
                while not offsetCalibration:  ### & other
                        leftHipOffset, leftKneeOffset, leftHipOutputAngleDesired, leftKneeOutputAngleDesired, leftOffsetCalibration = calibrateJointReadings(leftLeg)
                        rightHipOffset, rightKneeOffset, rightHipOutputAngleDesired, rightKneeOutputAngleDesired, rightOffsetCalibration = calibrateJointReadings(rightLeg)
                        offsetCalibration = (leftOffsetCalibration and rightOffsetCalibration)
                        time.sleep(0.01)
                        leftHipOutputAngleCurrent, leftKneeOutputAngleCurrent = leftHipOutputAngleDesired, leftKneeOutputAngleDesired
                        rightHipOutputAngleCurrent, rightKneeOutputAngleCurrent = rightHipOutputAngleDesired, rightKneeOutputAngleDesired
                        if offsetCalibration:
                                globalStartTime = time.time()

                loopStartTime = time.time()
                elapsedTime = loopStartTime - globalStartTime
                timeSteps.append(elapsedTime)

                ######<<<<<< MAIN LOOP >>>>>>######
                leftHipRotorAngleDesired, leftKneeRotorAngleDesired = getRotorAngleRad(leftHipOutputAngleDesired - leftHipOffset), getRotorAngleRad(leftKneeOutputAngleDesired - leftKneeOffset)
                rightHipRotorAngleDesired, rightKneeRotorAngleDesired = getRotorAngleRad(rightHipOutputAngleDesired - rightHipOffset), getRotorAngleRad(rightKneeOutputAngleDesired - rightKneeOffset)
                kpRotorHip, kdRotorHip, kpRotorKnee, kdRotorKnee = chooseRotorGains(crouching)

                ###<<< LEFT HIP >>>###
                data = sendCmdRcvData(leftLeg, id.hip, kpRotorHip, kdRotorHip, leftHipRotorAngleDesired, 0.0, hipTau)
                leftHipOutputAngleCurrent = getOutputAngleDeg(data.q) + leftHipOffset
                hipTorque = calculateOutputTorque(kpRotorHip, leftHipRotorAngleDesired, data.q, kdRotorHip, 0.0, data.dq, hipTau)
                outputData(leftLeg, id.hip, data.q, leftHipOffset, data.dq, hipTorque, data.temp, data.merror,hipTau)
                leftHipOutputAngles.append(leftHipOutputAngleCurrent), leftHipCommandAngles.append(leftHipOutputAngleDesired), leftHipOutputTorque.append(hipTorque)

                ###<<< RIGHT HIP >>>###
                data = sendCmdRcvData(rightLeg, id.hip, kpRotorHip, kdRotorHip, rightHipRotorAngleDesired, 0.0, hipTau)
                rightHipOutputAngleCurrent = getOutputAngleDeg(data.q) + rightHipOffset
                hipTorque = calculateOutputTorque(kpRotorHip, rightHipRotorAngleDesired, data.q, kdRotorHip, 0.0, data.dq, hipTau), outputData(rightLeg, id.hip, data.q, rightHipOffset, data.dq, hipTorque, data.temp, data.merror, hipTau)
                rightHipOutputAngles.append(rightHipOutputAngleCurrent), rightHipCommandAngles.append(rightHipOutputAngleDesired), rightHipOutputTorque.append(hipTorque)

                ###<<< LEFT KNEE >>>###
                data = sendCmdRcvData(leftLeg, id.knee, kpRotorKnee, kdRotorKnee, leftKneeRotorAngleDesired, 0.0, kneeTau)
                leftKneeOutputAngleCurrent = getOutputAngleDeg(data.q) + leftKneeOffset
                kneeTorque = calculateOutputTorque(kpRotorKnee, leftKneeRotorAngleDesired, data.q, kdRotorKnee, 0.0, data.dq, kneeTau)
                outputData(leftLeg, id.knee, data.q, leftKneeOffset, data.dq, kneeTorque, data.temp, data.merror, kneeTau)
                leftKneeOutputAngles.append(leftKneeOutputAngleCurrent), leftKneeCommandAngles.append(leftKneeOutputAngleDesired), leftKneeOutputTorque.append(kneeTorque)

                ###<<< RIGHT KNEE >>>###
                data = sendCmdRcvData(rightLeg, id.knee, kpRotorKnee, kdRotorKnee, rightKneeRotorAngleDesired, 0.0, kneeTau)
                rightKneeOutputAngleCurrent = getOutputAngleDeg(data.q) + rightKneeOffset
                kneeTorque = calculateOutputTorque(kpRotorKnee, rightKneeRotorAngleDesired, data.q, kdRotorKnee, 0.0, data.dq, kneeTau)
                outputData(rightLeg, id.knee, data.q, rightKneeOffset, data.dq, kneeTorque, data.temp, data.merror, kneeTau)
                rightKneeOutputAngles.append(rightKneeOutputAngleCurrent), rightKneeCommandAngles.append(rightKneeOutputAngleDesired), rightKneeOutputTorque.append(kneeTorque)

                ###<<< CROUCHING CONTROL >>>###
                crouchHeightDesiredNew = getCrouchCommand(pygame.event.get(), crouchHeightDesiredNew, crouchIncrement)
                leftHipOutputAngleDesired, leftKneeOutputAngleDesired, crouchHeightDesiredPrev, crouching \
                        = crouchControl('front',leftHipOutputAngleCurrent, leftKneeOutputAngleCurrent, crouchHeightDesiredPrev, crouchHeightDesiredNew, crouchDuration, crouching)

                rightHipOutputAngleDesired, rightKneeOutputAngleDesired, crouchHeightDesiredPrev, crouching \
                        = crouchControl('front', rightHipOutputAngleCurrent, rightKneeOutputAngleCurrent, crouchHeightDesiredPrev, crouchHeightDesiredNew, crouchDuration, crouching)

                ###<<< LOOP TIMING >>>###
                loopTime = time.time() - loopStartTime
                print(f"Loop Time: {loopTime}\n")

except KeyboardInterrupt:
        ### Command everything to 0?
        print("\nLoop stopped by user. Saving figure...")
        #try:
                ### ADD SERIAL INPUT TO DIFFERENTIATE LEFT AND RIGHT
                #plotAndSaveLegData(leftLeg, timeSteps, leftHipOutputAngles, leftHipCommandAngles, leftHipOutputTorque, leftKneeOutputAngles, leftKneeCommandAngles, leftKneeOutputTorque, crouchDuration)
                #plotAndSaveLegData(rightLeg, timeSteps, rightHipOutputAngles, rightHipCommandAngles, rightHipOutputTorque, rightKneeOutputAngles, rightKneeCommandAngles, rightKneeOutputTorque, crouchDuration)
                #print(f"Error encountered while saving figure: {e}")
        #finally:
        sys.exit(0)  # Ensure clean exit
