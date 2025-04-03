import time
import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append('../lib')
from unitree_actuator_sdk import *


serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()


data.motorType = MotorType.A1
cmd.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1,MotorMode.FOC)
cmd.id   = 0
serial.sendRecv(cmd, data)
hipInitial = ((data.q /queryGearRatio(MotorType.A1)) * (180 / np.pi))
offsetHip  = -90.0 - hipInitial


data.motorType = MotorType.A1
cmd.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1,MotorMode.FOC)
cmd.id   = 1
serial.sendRecv(cmd, data)
kneeInitial = ((data.q /queryGearRatio(MotorType.A1)) * (180 / np.pi))
offsetKnee  = 0.0 - kneeInitial

calibration = False

# Data storage for plotting
hipOutputAnglesDeg = []
kneeOutputAnglesDeg = []


timeSteps = []

globalStartTime = time.time()

try:
    while True:

            startTime = time.time()
            elapsedTime = startTime - globalStartTime
            timeSteps.append(elapsedTime)

            data.motorType = MotorType.A1
            cmd.motorType = MotorType.A1
            cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
            cmd.id = 0
            cmd.q = 0.0
            cmd.dq = 0.0  # 6.28*queryGearRatio(MotorType.A1)
            cmd.kp = 0.0
            cmd.kd = 0.0
            cmd.tau = 0.0
            serial.sendRecv(cmd, data)

            Angle = ((data.q / queryGearRatio(MotorType.A1)) * (180 / np.pi))
            hipOutputAnglesDeg.append(Angle)

            print('\n')
            print("Raw Output Angle (Hip) rad: " + str((data.q / queryGearRatio(MotorType.A1))))
            print("Raw Output Angle (Hip) deg: " + str(Angle))
            print(f"ISSUE? {data.merror}")
            print('\n')

            time.sleep(0.02)  # 200 us
            '''
            data.motorType = MotorType.A1
            cmd.motorType = MotorType.A1
            cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
            cmd.id = 1
            cmd.q = 0.0
            cmd.dq = 0.0  # 6.28*queryGearRatio(MotorType.A1)
            cmd.kp = 0.0
            cmd.kd = 0.0
            cmd.tau = 0.0
            serial.sendRecv(cmd, data)

            Angle = ((data.q / queryGearRatio(MotorType.A1)) * (180 / np.pi))
            kneeOutputAnglesDeg.append(Angle)

            print('\n')
            print("Raw Output Angle (Knee) rad: " + str((data.q / queryGearRatio(MotorType.A1))))
            print("Raw Output Angle (Knee) deg: " + str(Angle))
            print(f"ISSUE? {data.merror}")
            print('\n')
            time.sleep(0.02)  # 200 us
            '''
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