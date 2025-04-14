import time
import sys
import numpy as np
import matplotlib.pyplot as plt
from functions import *
sys.path.append('../lib')
from unitree_actuator_sdk import *

PI = np.pi

serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()

gearRatio = queryGearRatio(MotorType.A1)

output_kp = 25
output_kd = 0.6
rotor_kp = 0
rotor_kd = 0

rotor_kp = (output_kp / (gearRatio * gearRatio)) / 26.07
rotor_kd = (output_kd / (gearRatio * gearRatio)) * 100.0

data.motorType = MotorType.A1
cmd.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1,MotorMode.FOC)
cmd.id   = 0
serial.sendRecv(cmd, data)
hipInitial = ((data.q /queryGearRatio(MotorType.A1)) * (180 / np.pi))
offsetHip  = -90.0 - hipInitial
output_angle_c0 = (data.q / gearRatio) * (180 / PI)

data.motorType = MotorType.A1
cmd.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1,MotorMode.FOC)
cmd.id   = 1
serial.sendRecv(cmd, data)
kneeInitial = ((data.q /queryGearRatio(MotorType.A1)) * (180 / np.pi))
offsetKnee  = 0.0 - kneeInitial
output_angle_c1 = (data.q / gearRatio) * (180 / PI)
calibration = False

# Data storage for plotting
hipOutputAnglesDeg = []
kneeOutputAnglesDeg = []

HipAngle = 0

timeSteps = []

globalStartTime = time.time()


sin_counter = 0.0

try:
    while True:
            sin_counter += 0.001
            output_angle_d0 = output_angle_c0 + 20 * np.sin(2 * PI * sin_counter)
            rotor_angle_d0 = (output_angle_d0 * (PI / 180)) * gearRatio

            output_angle_d1 = output_angle_c1 + 20 * np.sin(2 * PI * sin_counter)
            rotor_angle_d1 = (output_angle_d1 * (PI / 180)) * gearRatio

            startTime = time.time()
            elapsedTime = startTime - globalStartTime
            timeSteps.append(elapsedTime)

            data.motorType = MotorType.A1
            cmd.motorType = MotorType.A1
            cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
            cmd.id = 0
            cmd.q = rotor_angle_d0
            cmd.dq = 0.0  # 6.28*queryGearRatio(MotorType.A1)
            cmd.kp = rotor_kp
            cmd.kd = rotor_kd
            cmd.tau = 0.0
            if serial.sendRecv(cmd, data):
                HipAngle = ((data.q / queryGearRatio(MotorType.A1)) * (180 / np.pi))
            else:
                HipAngle = HipAngle
                print('Waiting for Hip motor to respond')
            hipOutputAnglesDeg.append(HipAngle)

            print('\n')
            print("Raw Output Angle (Hip) rad: " + str((data.q / queryGearRatio(MotorType.A1))))
            print("Raw Output Angle (Hip) deg: " + str(HipAngle))
            print(f"ISSUE? {data.merror}")
            print('\n')

            time.sleep(0.02)  # 200 us

            data.motorType = MotorType.A1
            cmd.motorType = MotorType.A1
            cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
            cmd.id = 1
            cmd.q = rotor_angle_d1
            cmd.dq = 0.0  # 6.28*queryGearRatio(MotorType.A1)
            cmd.kp = rotor_kp
            cmd.kd = rotor_kd
            cmd.tau = 0.0
            while not serial.sendRecv(cmd, data):
                print('Waiting for Knee motor to respond')


            Angle = ((data.q / queryGearRatio(MotorType.A1)) * (180 / np.pi))
            kneeOutputAnglesDeg.append(Angle)

            print('\n')
            print("Raw Output Angle (Knee) rad: " + str((data.q / queryGearRatio(MotorType.A1))))
            print("Raw Output Angle (Knee) deg: " + str(Angle))
            print(f"ISSUE? {data.merror}")
            print('\n')
            time.sleep(0.2)  # 200 us

except KeyboardInterrupt:
        print("\nLoop stopped by user. Saving figure...")
        try:
            # Ensure all lists have the same length
            min_length = min(len(timeSteps), len(hipOutputAnglesDeg), len(kneeOutputAnglesDeg))

            if min_length == 0:
                print("No data collected. Exiting without saving.")
                sys.exit(0)  # Exit safely if no data

            timeSteps = timeSteps[:min_length]
            hipOutputAngles = hipOutputAnglesDeg[:min_length]
            kneeOutputAngles = kneeOutputAnglesDeg[:min_length]

            # Plotting
            plt.figure()
            plt.plot(timeSteps, hipOutputAngles, label='Hip Output Angles')
            plt.plot(timeSteps, kneeOutputAngles, label='Knee Output Angles')

            plt.xlabel('Time (s)')
            plt.ylabel('Angle (deg)')
            plt.title('Hip and Knee Angles Over Time')
            plt.legend(loc='best')
            plt.grid()

            # Save the figure before exiting
            plt.savefig("JointAnglesOverTime.png", dpi=300)
            print("Figure saved as JointAngleOverTime.png")

            # Close the plot
            plt.close()
        except Exception as e:
                print(f"Error encountered while saving figure: {e}")
        finally:
                sys.exit(0)  # Ensure clean exit