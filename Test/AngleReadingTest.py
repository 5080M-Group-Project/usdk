import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import os
import csv
from functions import *
sys.path.append('../lib')
from unitree_actuator_sdk import *

PI = np.pi

serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()

gearRatio = queryGearRatio(MotorType.A1)


data.motorType = MotorType.A1
cmd.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1,MotorMode.FOC)
cmd.id   = 0
serial.sendRecv(cmd, data)
hipInitial = ((data.q /queryGearRatio(MotorType.A1)) * (180 / np.pi))
offsetHip  = -90.0 - hipInitial
output_angle_c0 = hipInitial

data.motorType = MotorType.A1
cmd.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1,MotorMode.FOC)
cmd.id   = 1
serial.sendRecv(cmd, data)
kneeInitial = ((data.q /queryGearRatio(MotorType.A1)) * (180 / np.pi))
offsetKnee  = 0.0 - kneeInitial
output_angle_c1 = kneeInitial
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
            while not serial.sendRecv(cmd, data):
                print('Waiting for Knee motor to respond')
            Angle = ((data.q / queryGearRatio(MotorType.A1)) * (180 / np.pi))
            hipOutputAnglesDeg.append(Angle)

            print('\n')
            print("Raw Output Angle (Hip) rad: " + str((data.q / queryGearRatio(MotorType.A1))))
            print("Raw Output Angle (Hip) deg: " + str(Angle))
            print(f"ISSUE? {data.merror}")
            print('\n')

            time.sleep(0.02)  # 200 us

            data.motorType = MotorType.A1
            cmd.motorType = MotorType.A1
            cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
            cmd.id = 1
            cmd.q = 0
            cmd.dq = 0.0  # 6.28*queryGearRatio(MotorType.A1)
            cmd.kp = 0.0
            cmd.kd = 0.0
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

            # Base name for figure and CSV
            base_name = "JointAnglesOverTime"

            # Save the figure
            figure_filename = f"{base_name}.png"
            plt.savefig(figure_filename, dpi=300)
            print(f"Figure saved as {figure_filename}")

            # Save the data as CSV
            csv_filename = f"{base_name}.csv"
            with open(csv_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Time (s)', 'Hip Angle (deg)', 'Knee Angle (deg)'])
                for t, hip, knee in zip(timeSteps, hipOutputAngles, kneeOutputAngles):
                    writer.writerow([t, hip, knee])

            print(f"Data saved as {csv_filename}")

            plt.close()
        except Exception as e:
                print(f"Error encountered while saving figure: {e}")
        finally:
                sys.exit(0)  # Ensure clean exit