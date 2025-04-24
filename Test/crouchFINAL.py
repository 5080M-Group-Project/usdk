
from functions import *

######## NEEDED??? ########
sys.path.append('../lib')
from unitree_actuator_sdk import *
############################

serial = SerialPort('/dev/ttyUSB0')

##### NOTE 1: All rotor angles in RAD, all output angles in DEG########
##### NOTE 2: Whenever reading angles +offset, whenever commanding -offset. Offset in DEG######

#Initialize Loop Variables

# Data storage for plotting
hipOutputAngles, hipCommandAngles, hipOutputTorque, kneeOutputAngles, kneeCommandAngles, kneeOutputTorque, timeSteps = [], [], [], [], [], [], []

#NEEDED?
hipOffset, kneeOffset = 0.0, 0.0
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
                while not offsetCalibration: ### & other
                        hipOffset, kneeOffset, hipOutputAngleDesired, kneeOutputAngleDesired, offsetCalibration = calibrateJointReadings(serial)
                        time.sleep(0.1)
                        hipOutputAngleCurrent, kneeOutputAngleCurrent = hipOutputAngleDesired, kneeOutputAngleDesired
                        if offsetCalibration:
                                globalStartTime = time.time()

                loopStartTime = time.time()
                elapsedTime = loopStartTime - globalStartTime
                timeSteps.append(elapsedTime)

                ######<<<<<< MAIN LOOP >>>>>>######
                hipRotorAngleDesired, kneeRotorAngleDesired = getRotorAngleRad(hipOutputAngleDesired - hipOffset), getRotorAngleRad(kneeOutputAngleDesired - kneeOffset)
                kpRotorHip, kdRotorHip, kpRotorKnee, kdRotorKnee = chooseRotorGains(crouching)

                print(f"🛠️ Raw Hip Rotor Angle Command (rad): {hipRotorAngleDesired:.4f}")
                print(f"🛠️ Raw Knee Rotor Angle Command (rad): {kneeRotorAngleDesired:.4f}")

                ###<<< HIP >>>###
                data = sendCmdRcvData(serial, id.hip, kpRotorHip, kdRotorHip, hipRotorAngleDesired, 0.0, hipTau)
                hipOutputAngleCurrent = (getOutputAngleDeg(data.q) + hipOffset)
                #hipTau = data.tau
                hipTorque = calculateOutputTorque(kpRotorHip, hipRotorAngleDesired, data.q, kdRotorHip, 0.0, data.dq, hipTau)
                outputData(serial, id.hip, data.q, hipOffset, data.dq, hipTorque, data.temp, data.merror, hipTau)
                hipOutputAngles.append(hipOutputAngleCurrent), hipCommandAngles.append(hipOutputAngleDesired), hipOutputTorque.append(hipTorque)

                ###<<< KNEE >>>###
                data = sendCmdRcvData(serial, id.knee, kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired, 0.0, kneeTau)
                kneeOutputAngleCurrent = getOutputAngleDeg(data.q) + kneeOffset
                #kneeTau = data.tau
                kneeTorque = calculateOutputTorque(kpRotorKnee, kneeRotorAngleDesired, data.q, kdRotorKnee, 0.0, data.dq, kneeTau)
                outputData(serial, id.knee, data.q, kneeOffset, data.dq, kneeTorque, data.temp, data.merror, kneeTau)
                kneeOutputAngles.append(kneeOutputAngleCurrent), kneeCommandAngles.append(kneeOutputAngleDesired), kneeOutputTorque.append(kneeTorque)

                ###<<< CROUCHING CONTROL >>>###
                crouchHeightDesiredNew = getCrouchCommand(pygame.event.get(),crouchHeightDesiredNew, crouchIncrement)
                hipOutputAngleDesired, kneeOutputAngleDesired, crouchHeightDesiredPrev, crouching \
                        = crouchControl('front', hipOutputAngleCurrent, kneeOutputAngleCurrent, crouchHeightDesiredPrev, crouchHeightDesiredNew, crouchDuration, crouching)

                ###<<< LOOP TIMING >>>###
                loopTime = time.time() - loopStartTime
                print(f"Loop Time: {loopTime}\n")

except KeyboardInterrupt:
        ### Command everything to 0?
        print("\nLoop stopped by user. Saving figure...")
        #try:
                ### ADD SERIAL INPUT TO DIFFERENTIATE LEFT AND RIGHT
                #plotAndSaveLegData(timeSteps,hipOutputAngles, hipCommandAngles, hipOutputTorque, kneeOutputAngles, kneeCommandAngles, kneeOutputTorque, crouchDuration)
               # print(f"Error encountered while saving figure: {e}")
        #finally:
        sys.exit(0)  # Ensure clean exit
