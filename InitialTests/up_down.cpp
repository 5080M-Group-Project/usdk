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

  cmd.motorType = MotorType::A1;
  data.motorType = MotorType::A1;
  cmd.mode  = queryMotorMode(MotorType::A1,MotorMode::FOC);
  cmd.id    = 0;
  cmd.kp    = 0.0;
  cmd.kd    = 0.0;
  cmd.q     = 0.0;
  cmd.dq    = 0.0;
  cmd.tau   = 0.0;
  serial.sendRecv(&cmd,&data);  cmd.motorType = MotorType::A1;
  float init_angle_c0 = data.q*PI/180;
  float output_angle_c0;
  output_angle_c0 = (data.q / gear_ratio) * (180/PI);
  
  data.motorType = MotorType::A1;
  cmd.mode  = queryMotorMode(MotorType::A1,MotorMode::FOC);
  cmd.id    = 1;
  cmd.kp    = 0.0;
  cmd.kd    = 0.0;
  cmd.q     = 0.0;
  cmd.dq    = 0.0;
  cmd.tau   = 0.0;
  serial.sendRecv(&cmd,&data);
  float init_angle_c1 = data.q*PI/180;
  float output_angle_c1;
  output_angle_c1 = (data.q / gear_ratio) * (180/PI);
  
  
  while(true) {
    sin_counter+=0.0025;
    float output_angle_d0;
    output_angle_d0 = output_angle_c0 + 45*sin(2*PI*sin_counter);
    float rotor_angle_d0 = (output_angle_d0 * (PI/180)) * gear_ratio;
    
    float output_angle_d1;
    output_angle_d1 = output_angle_c1 - 30*sin(2*PI*sin_counter);
    float rotor_angle_d1 = (output_angle_d1 * (PI/180)) * gear_ratio;
    
    cmd.motorType = MotorType::A1;
    data.motorType = MotorType::A1;
    cmd.mode  = queryMotorMode(MotorType::A1,MotorMode::FOC);
    cmd.id    = 1;
    cmd.kp    = rotor_kp;
    cmd.kd    = rotor_kd;
    cmd.q     = rotor_angle_d0;
    cmd.dq    = 0.0;
    cmd.tau   = 0.0;
    serial.sendRecv(&cmd,&data);

    cmd.motorType = MotorType::A1;
    data.motorType = MotorType::A1;
    cmd.mode  = queryMotorMode(MotorType::A1,MotorMode::FOC);
    cmd.id    = 0;
    cmd.kp    = rotor_kp;
    cmd.kd    = rotor_kd;
    cmd.q     = rotor_angle_d1*2;
    cmd.dq    = 0.0;
    cmd.tau   = 0.0;
    serial.sendRecv(&cmd,&data);

    std::cout <<  std::endl;
    std::cout <<  "motor.q: "    << data.q * (180/PI)/ gear_ratio    <<  std::endl;
    std::cout <<  "motor.temp: "   << data.temp   <<  std::endl;
    std::cout <<  "motor.W: "      << data.dq / gear_ratio     <<  std::endl;
    std::cout <<  "motor.merror: " << data.merror <<  std::endl;
    std::cout <<  "rotor_kp: " << rotor_kp <<  std::endl;
    std::cout <<  "rotor_kd: " << rotor_kd <<  std::endl;
    std::cout <<  std::endl;
    
    cmd.motorType = MotorType::A1;
    data.motorType = MotorType::A1;
    cmd.mode  = queryMotorMode(MotorType::A1,MotorMode::FOC);
    cmd.id    = 2;
    cmd.kp    = 0.0;
    cmd.kd    = 2;
    cmd.q     = 0.0;
    cmd.dq    = 0.0;//-6.28*queryGearRatio(MotorType::A1);
    cmd.tau   = 0.0;
    serial.sendRecv(&cmd,&data);

    usleep(200);
  }

}//KNEE: neutral position is 0.5, so range is -2 and 3 

//HIP: neutral position is 0.3, so range is and -1.4 and 2  //cmd.q = 0.0;


