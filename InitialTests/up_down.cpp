#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <math.h>
#define PI 3.1415926

int main() {

  SerialPort  serial("/dev/ttyUSB0");
  MotorCmd    cmd;
  MotorData   data;

  float output_kp = 2.5; //25
  float output_kd = 0.2;
  float rotor_kp = 0;
  float rotor_kd = 0;
  float gear_ratio = queryGearRatio(MotorType::A1);
  float sin_counter = 0.0;

  float L1 = 0.165;    // Example length 1
  float L2 = 0.165;    // Example length 2
  float xdes = 0.0;  // Desired x coordinate
  float ydes = 0.15;  // Desired y coordinate

    // Use pow() for exponentiation
  float beta = acos((pow(L1, 2) + pow(L2, 2) - pow(xdes, 2) - pow(ydes, 2)) / (2 * L1 * L2));
  float alpha = acos((pow(xdes, 2) + pow(ydes, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * sqrt(pow(xdes, 2) + pow(ydes, 2))));
  float gamma = atan2(ydes, xdes);  // atan2(y, x) is correct

    // Correct multiplication (no dereferencing needed)
  float thetaHip = (gamma - alpha) * (180.0 / PI);
  float thetaKnee = (PI - beta) * (180.0 / PI);

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
  float output_angle_c1;
  output_angle_c1 = (data.q / gear_ratio) * (180/PI);
  
  
  while(true) {
    sin_counter+=0.005;
    float output_angle_d0;
    output_angle_d0 =  output_angle_c0 -  (90-thetaHip)*sin(2*PI*sin_counter); //change thetaHip to 30
    float rotor_angle_d0 = (output_angle_d0 * (PI/180)) * gear_ratio;
    
    cmd.motorType = MotorType::A1;
    data.motorType = MotorType::A1;
    cmd.mode  = queryMotorMode(MotorType::A1,MotorMode::FOC);
    cmd.id    = 0;
    cmd.kp    = rotor_kp;
    cmd.kd    = rotor_kd;
    cmd.q     = rotor_angle_d0;
    cmd.dq    = 0.0;
    cmd.tau   = 0.0;
    serial.sendRecv(&cmd,&data);

    float output_angle_d1;
    output_angle_d1 = output_angle_c1 + thetaKnee*sin(2*PI*sin_counter); //thetaKnee to 2*30
    float rotor_angle_d1 = (output_angle_d1 * (PI/180)) * gear_ratio;

    cmd.motorType = MotorType::A1;
    data.motorType = MotorType::A1;
    cmd.mode  = queryMotorMode(MotorType::A1,MotorMode::FOC);
    cmd.id    = 1;
    cmd.kp    = rotor_kp;
    cmd.kd    = rotor_kd;
    cmd.q     = rotor_angle_d1;
    cmd.dq    = 0.0;
    cmd.tau   = 0.0;
    serial.sendRecv(&cmd,&data);

    std::cout <<  std::endl;
    std::cout <<  "motor.q: "    << data.q * (180/PI)/ gear_ratio    <<  std::endl;
    std::cout <<  std::endl;
    usleep(250);
  }

}//KNEE: neutral position is 0.5, so range is -2 and 3 

//HIP: neutral position is 0.3, so range is and -1.4 and 2  //cmd.q = 0.0;


