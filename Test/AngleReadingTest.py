import time
import sys
import numpy as np
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
offsetHip  = 90.0 - hipInitial


data.motorType = MotorType.A1
cmd.motorType = MotorType.A1
cmd.mode = queryMotorMode(MotorType.A1,MotorMode.FOC)
cmd.id   = 1
serial.sendRecv(cmd, data)
kneeInitial = ((data.q /queryGearRatio(MotorType.A1)) * (180 / np.pi))
offsetKnee  = 0.0 - kneeInitial


while True:
    data.motorType = MotorType.A1
    cmd.motorType = MotorType.A1
    cmd.mode = queryMotorMode(MotorType.A1,MotorMode.FOC)
    cmd.id   = 0
    cmd.q    = 0.0
    cmd.dq   = 0.0 #6.28*queryGearRatio(MotorType.A1)
    cmd.kp   = 0.0
    cmd.kd   = 0.0
    cmd.tau  = 0.0
    serial.sendRecv(cmd, data)

    Angle = ((data.q / queryGearRatio(MotorType.A1)) * (180 / np.pi)) + offsetHip

    print('\n')
    print("Raw Output Angle (Hip) rad: " + str((data.q / queryGearRatio(MotorType.A1))))
    print("Raw Output Angle (Hip) deg: " + str((data.q / queryGearRatio(MotorType.A1))*(180 / np.pi)))
    print("Angle w. offset (Hip) deg: " + str(Angle))
    print("Initial (Hip) deg: " + str(hipInitial))
    print("Offset (Hip) deg: " + str(offsetHip))
    print('\n')
    time.sleep(0.0002) # 200 us

    data.motorType = MotorType.A1
    cmd.motorType = MotorType.A1
    cmd.mode = queryMotorMode(MotorType.A1,MotorMode.FOC)
    cmd.id   = 1
    cmd.q    = 0.0
    cmd.dq   = 0.0 #6.28*queryGearRatio(MotorType.A1)
    cmd.kp   = 0.0
    cmd.kd   = 0.0
    cmd.tau  = 0.0
    serial.sendRecv(cmd, data)

    Angle = ((data.q / queryGearRatio(MotorType.A1)) * (180 / np.pi)) + offsetKnee

    print('\n')
    print("Raw Output Angle (Knee) rad: " + str((data.q / queryGearRatio(MotorType.A1))))
    print("Raw Output Angle (Knee) deg: " + str((data.q / queryGearRatio(MotorType.A1))*(180 / np.pi)))
    print("Angle w. offset (Knee) deg: " + str(Angle))
    print("Initial (Knee) deg: " + str(kneeInitial))
    print("Offset (Knee) deg: " + str(offsetKnee))
    print('\n')
    time.sleep(0.0002) # 200 us