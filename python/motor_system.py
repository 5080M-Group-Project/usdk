import time
import sys
sys.path.append('../lib')
from unitree_actuator_sdk import *


serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()