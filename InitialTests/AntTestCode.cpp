#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <math.h>
#define PI 3.1415926

int main() {

  SerialPort  serial("/dev/ttyUSB0");
  MotorCmd    cmd;
  MotorData   data;

  float output_kp = 25;
  float output_kd = 0.6;
  float rotor_kp = 0;
  float rotor_kd = 0;
  float gear_ratio = queryGearRatio(MotorType::A1);
  float sin_counter = 0.0;

  rotor_kp = (output_kp / (gear_ratio * gear_ratio)) / 26.07;
  rotor_kd = (output_kd / (gear_ratio * gear_ratio)) * 100.0;

  cmdActuator(0,0.0,0.0,0.0,0.0,0.0);

  cmdActuator(1,0.0,0.0,0.0,0.0,0.0);
  
  cmdActuator(2,0.0,0.0,0.0,0.0,0.0);


  outputAngleCurrent = getCurrentOutputAngle();
  while(true) 
  {
    

    cmdActuator(0,hip_rotor_kp,hip_rotor_kd,hip_rotor_angle_d,0.0,0.0);
    outputData(0);

    cmdActuator(1,knee_rotor_kp,hip_rotor_kd,knee_rotor_angle_d,0.0,0.0);
    outputData(1);

    cmdActuator(2,wheel_rotor_kp,wheel_rotor_kd,0.0,wheel_rotor_angular_velocity,0.0);
    outputData(2);

    usleep(200);
  }

}

float getCurrentOutputAngle(){
     float outputAngleCurrent;
     outputAngleCurrent = (data.q / gear_ratio) * (180/PI);
     return outputAngleCurrent
}
    

float getRotorAngle(outputAngleCurrent){
    sin_counter+=0.0001;
    float outputAngleDesired;
    outputAngleDesired = outputAngleCurrent + 90 * sin(2*PI*sin_counter);
    float rotorAngleDesired = (outputAngleDesired * (PI/180)) * gear_ratio;
}

void cmdActuator(id,kp,kd,q,dq,tau){
  cmd.motorType = MotorType::A1;
  data.motorType = MotorType::A1;
  cmd.mode  = queryMotorMode(MotorType::A1,MotorMode::FOC);
  cmd.id    = 0;
  cmd.kp    = 0.0;
  cmd.kd    = 0.0;
  cmd.q     = 0.0;
  cmd.dq    = 0.0;
  cmd.tau   = 0.0;
  serial.sendRecv(&cmd,&data);
}

void outputData(id){
  std::cout <<  std::endl;
  std::cout <<  "motor.ID: "    << id  <<  std::endl;
  std::cout <<  "motor.q: "    << data.q / gear_ratio    <<  std::endl;
  std::cout <<  "motor.temp: "   << data.temp   <<  std::endl;
  std::cout <<  "motor.W: "      << data.dq / gear_ratio     <<  std::endl;
  std::cout <<  "motor.merror: " << data.merror <<  std::endl;
  std::cout <<  "rotor_kp: " << rotor_kp <<  std::endl;
  std::cout <<  "rotor_kd: " << rotor_kd <<  std::endl;
  std::cout <<  std::endl;
}