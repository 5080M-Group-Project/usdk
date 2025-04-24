import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import os
import csv
#from functions import *
sys.path.append('../lib')
from unitree_actuator_sdk import *

PI = np.pi

serial = SerialPort('/dev/ttyUSB0')
serial2 = SerialPort('/dev/ttyUSB1')
cmd = MotorCmd()
data = MotorData()

gearRatio = queryGearRatio(MotorType.A1)

output_kp = 0.0
output_kd = 2.0
rotor_kp = 0
rotor_kd = 0

rotor_kp = (output_kp / (gearRatio * gearRatio)) / 26.07
rotor_kd = (output_kd / (gearRatio * gearRatio)) * 100.0


data.motorType = MotorType.A1
cmd.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1,MotorMode.FOC)
cmd.id   = 2
serial.sendRecv(cmd, data)
kneeInitial = ((data.q /queryGearRatio(MotorType.A1)) * (180 / np.pi))
offsetKnee  = 0.0 - kneeInitial
output_angle_c1 = kneeInitial
calibration = False

# Data storage for plotting
hipOutputAnglesDeg = []
kneeOutputAnglesDeg = []
hipCommandAngles = []
kneeCommandAngles = []

HipAngle, Angle = 0, 0

timeSteps = []

globalStartTime = time.time()

sin_counter = 0.0

try:
    while True:
            while output_angle_c1 > 180:
                cmd.id = 2
                serial.sendRecv(cmd, data)
                kneeInitial = ((data.q / queryGearRatio(MotorType.A1)) * (180 / np.pi))
                output_angle_c1 = kneeInitial

            '''
            output_angle_d1 = output_angle_c1 + 90
            rotor_angle_d1 = (output_angle_d1 * (PI / 180)) * gearRatio
            kneeCommandAngles.append(output_angle_d1)
            '''

            startTime = time.time()
            elapsedTime = startTime - globalStartTime
            timeSteps.append(elapsedTime)

            data.motorType = MotorType.A1
            cmd.motorType = MotorType.A1
            cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
            cmd.id = 2
            cmd.q = 0.0 #rotor_angle_d1
            cmd.dq =  6.28*queryGearRatio(MotorType.A1)
            cmd.kp = rotor_kp
            cmd.kd = rotor_kd
            cmd.tau = 0.0
            while not serial.sendRecv(cmd, data):
                print('Waiting for Knee motor to respond')
            while not serial2.sendRecv(cmd, data):
                print('Waiting for Knee motor to respond')
            Angle = ((data.q / queryGearRatio(MotorType.A1)) * (180 / np.pi))
            kneeOutputAnglesDeg.append(Angle)

            print('\n')
            print("Raw Output Angle (Knee) rad: " + str((data.q / queryGearRatio(MotorType.A1))))
            print("Raw Output Angle (Knee) deg: " + str(Angle))
            print(f"Temperature {data.temp}")
            print(f"ISSUE? {data.merror}")
            print('\n')
            time.sleep(0.2)  # 200 us

except KeyboardInterrupt:
        print("\nLoop stopped by user. Saving figure...")
        try:
            # Ensure all lists have the same length
            #min_length = min(len(timeSteps), len(hipOutputAnglesDeg), len(kneeOutputAnglesDeg), len(hipCommandAngles), len(kneeCommandAngles))
            min_length = min(len(timeSteps), len(kneeOutputAnglesDeg), len(kneeCommandAngles))

            if min_length == 0:
                print("No data collected. Exiting without saving.")
                sys.exit(0)  # Exit safely if no data

            timeSteps = timeSteps[:min_length]
            #hipOutputAngles = hipOutputAnglesDeg[:min_length]
            kneeOutputAngles = kneeOutputAnglesDeg[:min_length]
            #hipCommandAngles = hipCommandAngles[:min_length]
            kneeCommandAngles = kneeCommandAngles[:min_length]

            # Plotting
            plt.figure()
            #plt.plot(timeSteps, hipOutputAngles, label='Hip Output Angles')
            plt.plot(timeSteps, kneeOutputAngles, label='Knee Output Angles')
            #plt.plot(timeSteps, hipCommandAngles, label='Hip Command Angle')
            plt.plot(timeSteps, kneeCommandAngles, label='knee Command Angle')

            plt.xlabel('Time (s)')
            plt.ylabel('Angle (deg)')
            #plt.title('Hip and Knee Angles Over Time')
            plt.title('Knee Angles Over Time')
            plt.legend(loc='best')
            plt.grid()

            # Base name for figure and CSV
            #base_name = "MovingJointAnglesOverTime"
            base_name = f"SingleCommand_kp_{output_kp:.1f}_kd_{output_kd:.1f}"

            # Save the figure
            figure_filename = f"{base_name}.png"
            plt.savefig(figure_filename, dpi=300)
            print(f"Figure saved as {figure_filename}")

            # Save the data as CSV
            csv_filename = f"{base_name}.csv"
            with open(csv_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                '''
                writer.writerow(['Time (s)', 'Hip Output Angle (deg)','Hip Command Angle (deg)', 'Knee Output Angle (deg)','Knee Command Angle (deg)',])
                for t, hip, hipCmd, knee, kneeCmd in zip(timeSteps, hipOutputAngles, hipCommandAngles,  kneeOutputAngles, kneeCommandAngles):
                    writer.writerow([t, hip, hipCmd, knee,kneeCmd])
                '''
                writer.writerow(['Time (s)', 'Knee Output Angle (deg)','Knee Command Angle (deg)', ])
                for t,  knee, kneeCmd in zip(timeSteps, kneeOutputAngles, kneeCommandAngles):
                    writer.writerow([t, knee, kneeCmd])

            print(f"Data saved as {csv_filename}")

            plt.close()
        except Exception as e:
                print(f"Error encountered while saving figure: {e}")
        finally:
                sys.exit(0)  # Ensure clean exit