import time
import numpy as np

class Leg_kinematics():
    def thigh(self,typ,th1):#right
        angle=th1
        tt= np.radians(angle)
        cc=78991*np.cos(tt)
        tan=-cc/78991
        rad=np.arctan(tan)
        deg=np.rad2deg(rad)
            
        th1=tt
        th2=deg+90
        th2=np.deg2rad(th2)
            
        print(th1)
            
        print(th2)
            
        if th1>=0:
            th3=np.arcsin( (((-707*np.sin(th2)) - (707*np.cos(th2)*np.cos(th1)))/1000))*-1-np.pi
                
        else:
            th3=np.arcsin( (((-707*np.sin(th2)) - (707*np.cos(th2)*np.cos(th1)))/1000))
            
        #fix
        th2=th2-(np.pi/4)
        th3=th3+(np.pi/2)
            
        print(th3)
        
        if "L"in typ:
            th1=th1*-1
            th2=th2*-1
            th3=th3*-1
        else:
            th1=th1
            th2=th2
            th3=th3
        return th1,th2,th3
    def calf(self,typ,knee_deg,ankle_deg): #left
        angle=knee_deg
        kangle=angle
        bar_long=100
        link_long=55
        calf_long=94
        calf_angle=40.08
            
        knee_angle=180-(angle+calf_angle)
        knee_angle=np.deg2rad(knee_angle)
        link_motor=(((calf_long**2)+(link_long**2))-(2*calf_long*link_long*np.cos(knee_angle)))**0.5
            
        t1=np.arccos(((link_motor**2)+(link_long**2)-(bar_long**2))/(2*link_long*link_motor))
        t2=np.arcsin(link_long/(link_motor/np.sin(knee_angle)))
            
        motor_init=0.85095
        motor_rad=(t1+t2)-motor_init
            
        #ankle
        angle=ankle_deg
        clink_long=60
        umotor_long=299.67
        ulink_long=303.556
        dmotor_long=205
        dlink_long=204.022
        flink_long=45
            
        ankle_angle=angle
        ankle_angle=np.deg2rad(ankle_angle)
        umotor_foot=(((flink_long**2)+(umotor_long**2))-(2*flink_long*umotor_long*np.cos(ankle_angle)))**0.5
            
        t1=np.arccos(((umotor_foot**2)+(umotor_long**2)-(flink_long**2))/(2*umotor_foot*umotor_long))
        t2=np.arccos(((clink_long**2)+(umotor_foot**2)-(ulink_long**2))/(2*clink_long*umotor_foot))
        motor_init=0.85095
        up_pitch=(t1+t2)-motor_init+(np.deg2rad(kangle))
        print(up_pitch)
        print(np.rad2deg(up_pitch))
            
        dmotor_foot=(((flink_long**2)+(dmotor_long**2))-(2*flink_long*dmotor_long*np.cos(ankle_angle)))**0.5
            
        t1=np.arccos(((dmotor_foot**2)+(dmotor_long**2)-(flink_long**2))/(2*dmotor_foot*dmotor_long))
        t2=np.arccos(((clink_long**2)+(dmotor_foot**2)-(dlink_long**2))/(2*clink_long*dmotor_foot))
        motor_init=0.70095
        down_pitch=(t1+t2)-motor_init
        print(down_pitch)
        print(np.rad2deg(down_pitch))
        
        if "L" in typ:
            motor_rad=motor_rad*-1
            omotor_rad=motor_rad*-1
            down_pitch=down_pitch*-1
            up_pitch=up_pitch
        else:
            motor_rad=motor_rad
            omotor_rad=motor_rad*-1
            down_pitch=down_pitch
            up_pitch=up_pitch*-1
        
        return motor_rad,omotor_rad,down_pitch,up_pitch
    
    def kinematics(self,inp):

        RRmotor_rad,RLmotor_rad,Rdown_pitch,Rup_pitch=self.calf("R",inp,90)
        Rth1,Rth2,Rth3=self.thigh("R",inp)
        LRmotor_rad,LLmotor_rad,Ldown_pitch,Lup_pitch=self.calf("L",inp,90)
        Lth1,Lth2,Lth3=self.thigh("L",inp)

        return [Rth1,Rth2,Rth3,Lth1,Lth2,Lth3,RRmotor_rad,RLmotor_rad,LRmotor_rad,LLmotor_rad,Rdown_pitch,Rup_pitch,Ldown_pitch,Lup_pitch]

if __name__ == '__main__':
    kin=Leg_kinematics()
    print(kin.kinematics(0))