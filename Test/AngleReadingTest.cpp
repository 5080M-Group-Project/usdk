#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <math.h>
#define PI 3.1415926

int main() {

  SerialPort  serial("/dev/ttyUSB0");
  MotorCmd    cmd;
  MotorData   data;

  float gear_ratio = queryGearRatio(MotorType::A1);

  cmd.motorType = MotorType::A1;
  data.motorType = MotorType::A1;
  cmd.mode  = queryMotorMode(MotorType::A1,MotorMode::FOC);
  cmd.id    = 0;
  serial.sendRecv(&cmd,&data);
  float hipInitial = (data.q / gear_ratio) * (180/PI);
  float offsetHip  = 90.0 - hipInitial;

  cmd.motorType = MotorType::A1;
  data.motorType = MotorType::A1;
  cmd.mode  = queryMotorMode(MotorType::A1,MotorMode::FOC);
  cmd.id    = 1;
  serial.sendRecv(&cmd,&data);
  float kneeInitial = (data.q / gear_ratio) * (180/PI);
  float offsetKnee = 0.0 - kneeInitial;


  while(true) {

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

    std::cout << std::endl;
    std::cout << "Raw Output Angle (Hip) rad: " << data.q/gear_ratio << std::endl;
    std::cout << "Raw Output Angle (Hip) deg: " << (data.q/gear_ratio)*(180/PI) << std::endl;
    std::cout << "Angle w. offset (Hip) deg: " << (data.q/gear_ratio)*(180/PI) + offsetHip << std::endl;
    std::cout << "Initial (Hip) deg: " << hipInitial << std::endl;
    std::cout << "Offset (Hip) deg: " << offsetHip << std::endl;
    std::cout << std::endl;

    cmd.motorType = MotorType::A1;
    data.motorType = MotorType::A1;
    cmd.mode  = queryMotorMode(MotorType::A1,MotorMode::FOC);
    cmd.id    = 1;
    cmd.kp    = 0.0;
    cmd.kd    = 0.0;
    cmd.q     = 0.0;
    cmd.dq    = 0.0;
    cmd.tau   = 0.0;
    serial.sendRecv(&cmd,&data);

    std::cout << std::endl;
    std::cout << "Raw Output Angle (Knee) rad: " << data.q/gear_ratio << std::endl;
    std::cout << "Raw Output Angle (Knee) deg: " << (data.q/gear_ratio)*(180/PI) << std::endl;
    std::cout << "Angle w. offset (Knee) deg: " << (data.q/gear_ratio)*(180/PI) + offsetKnee << std::endl;
    std::cout << "Initial (Knee) deg: " << kneeInitial << std::endl;
    std::cout << "Offset (Knee) deg: " << offsetKnee << std::endl;
    std::cout << std::endl;

    usleep(250);
  }

}//KNEE: neutral position is 0.5, so range is -2 and 3

//HIP: neutral position is 0.3, so range is and -1.4 and 2  //cmd.q = 0.0;


