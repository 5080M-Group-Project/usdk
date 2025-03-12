import time
import sys
import json
import math
sys.path.append('../lib')
from unitree_actuator_sdk import *

import rclpy
import threading
from json import load

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        
        jsonFile = open('src/kh_system/kh_system/device_test2.json','r')
        self.device_json = json.load(jsonFile)
        self.part_name=list(self.device_json['port'].keys())
        
        
        self.serial_list=[]
        self.data_list=[]
        self.cmd_list=[]
        self.id_list=[]
           
        self.pos_now=[]
        self.pos_target=[]
        self.pos_start=[]   
        self.pos_move=[]
        self.step=[]
        
        self.motor_start()
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'pos',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.pos_target=list(msg.data)
        print(self.pos_target)
        self.motor_cmd()
    
    def motor_start(self):
        seq=0
        thread_lis=[]
        
        for part in self.part_name:
            for index in range(0,len(self.device_json[part]['name'])):
                name=f"{part}:{self.device_json[part]['name'][index]}"
                
                self.serial_list+=[SerialPort(self.device_json['port'][part])]
                self.cmd_list+=[MotorCmd()]
                self.data_list+=[MotorData()]
                self.id_list+=[self.device_json[part]["id"][index]]
                self.pos_now+=[0]
                self.pos_target+=[0]
                self.pos_start+=[0]
                self.pos_move+=[0]
                self.step+=[0]	
                
                
                thread_lis += [threading.Thread(target=self.motor_thread, args=(name,seq))]
                               
                seq+=1
                
        for thread in thread_lis:
            thread.start()  
                			
            
    def motor_point_set(self,name,seq):
    
        self.data_list[seq].motorType = MotorType.A1
        self.cmd_list[seq].motorType = MotorType.A1
        self.cmd_list[seq].mode = queryMotorMode(MotorType.A1,MotorMode.FOC)
        for x in range(0,50):
            self.cmd_list[seq].id   = self.id_list[seq]
            self.cmd_list[seq].q    = 0
            self.cmd_list[seq].dq   = 0
            self.cmd_list[seq].kp   = 0
            self.cmd_list[seq].kd   = 0
            self.cmd_list[seq].tau  = 0
            self.serial_list[seq].sendRecv(self.cmd_list[seq], self.data_list[seq])
        
            time.sleep(0.002) 
            now=self.data_list[seq].q/queryGearRatio(MotorType.A1)
            self.pos_start[seq]=now
        
        print(self.pos_start)
        print(f"{name} reset")
     
    def motor_cmd(self):

        times=1
        fps=10
        dely = times/fps
        
        
        for seq in range(0,len(self.pos_target)):
            self.step[seq]=((self.pos_target[seq]-self.pos_now[seq])/fps)
        
        
        for _ in range(0,fps):
            st=[]
            for seq in range(0,len(self.pos_target)):

                move= self.pos_now[seq] + self.step[seq]
                st+=[move]
            
            self.pos_move=st.copy()
            
            self.pos_now=st.copy()
                
        
        
    def motor_thread(self,name,seq):
        self.motor_point_set(name,seq)
        
        while True:
            self.cmd_list[seq].id   = self.id_list[seq]
            self.cmd_list[seq].q    = (self.pos_move[seq]+self.pos_start[seq])*queryGearRatio(MotorType.A1)
            self.cmd_list[seq].dq   = 0
            self.cmd_list[seq].kp   = 0.08
            self.cmd_list[seq].kd   = 15
            self.cmd_list[seq].tau  = 0
                

            self.serial_list[seq].sendRecv(self.cmd_list[seq], self.data_list[seq])
            
        
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
