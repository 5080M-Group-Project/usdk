#include <iostream>
#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <math.h>

#define PI 3.1415926

int main() {

    SerialPort serial("/dev/ttyUSB0");
    MotorCmd cmd;
    MotorData data;
    float gearRatio = queryGearRatio(MotorType::A1);

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

    // Initialize Hip Motor
    float kpOutHip = 2.5, kdOutHip = 0.2, kpRotorHip = 0.0, kdRotorHip = 0.0;
    kpRotorHip = (kpOutHip / (gearRatio * gearRatio)) / 26.07;
    kdRotorHip = (kdOutHip / (gearRatio * gearRatio)) * 100.0;
    cmd.motorType = MotorType::A1;
    data.motorType = MotorType::A1;
    cmd.mode  = queryMotorMode(MotorType::A1, MotorMode::FOC);
    cmd.id    = 0;
    cmd.kp    = 0.0; //proportional or position term. i.e. stiffness
    cmd.kd    = 0.0; //derivative or velocity term, i.e damping
    cmd.q     = 0.0; //angle, radians
    cmd.dq    = 0.0; //angular velocity, radians/s
    cmd.tau   = 0.0; //rotor feedforward torque
    serial.sendRecv(&cmd, &data);
    float hipAngleOutputInitial = (data.q / gearRatio) * (180 / PI);

    // Initialize Knee Motor
    float kpOutKnee = 2.5, kdOutKnee = 0.2, kpRotorKnee = 0.0, kdRotorKnee = 0.0;
    kpRotorKnee = (kpOutKnee / (gearRatio * gearRatio)) / 26.07;
    kdRotorKnee = (kdOutKnee / (gearRatio * gearRatio)) * 100.0;
    cmd.motorType = MotorType::A1;
    data.motorType = MotorType::A1;
    cmd.mode  = queryMotorMode(MotorType::A1, MotorMode::FOC);
    cmd.id    = 1;
    cmd.kp    = 0.0; //proportional or position term. i.e. stiffness
    cmd.kd    = 0.0; //derivative or velocity term, i.e damping
    cmd.q     = 0.0; //angle, radians
    cmd.dq    = 0.0; //angular velocity, radians/s
    cmd.tau   = 0.0; //rotor feedforward torque
    serial.sendRecv(&cmd, &data);
    float kneeAngleOutputInitial = (data.q / gearRatio) * (180 / PI);

    // Initialize Wheel Motor
    float kpOutWheel = 0.0, kdOutWheel = 2.0, kpRotorWheel = 0.0, kdRotorWheel = 0.0;
    kpRotorWheel = (kpOutWheel / (gearRatio * gearRatio)) / 26.07;
    kdRotorWheel = (kdOutWheel / (gearRatio * gearRatio)) * 100.0;
    cmd.motorType = MotorType::A1;
    data.motorType = MotorType::A1;
    cmd.mode  = queryMotorMode(MotorType::A1, MotorMode::FOC);
    cmd.id    = 2;
    cmd.kp    = 0.0; //proportional or position term. i.e. stiffness
    cmd.kd    = 0.0; //derivative or velocity term, i.e damping
    cmd.q     = 0.0; //angle, radians
    cmd.dq    = 0.0; //angular velocity, radians/s
    cmd.tau   = 0.0; //rotor feedforward torque
    serial.sendRecv(&cmd, &data);
    float wheelAngleOutputInitial = (data.q / gearRatio) * (180 / PI);

    //Initialize Loop Variables
    float torque = 0.0;
    float hipRotorAngleDesired = 0.0;
    float kneeRotorAngleDesired = 0.0;
    float wheelRotorAngularVelocityDesired = 0.0;
    float hipTau = 0.0;
    float kneeTau = 0.0;

    while (true) {
        sin_counter+=0.005;

        // Hip Motor Control
        //#####DETERMINING DESIRED ANGLE#######
        float hipOutputAngleDesired;
        hipOutputAngleDesired =  hipAngleOutputInitial -  (90-thetaHip)*sin(2*PI*sin_counter); //change thetaHip to 30
        float hipRotorAngleDesired = (hipOutputAngleDesired * (PI/180)) * gearRatio;

        cmd.motorType = MotorType::A1;
        data.motorType = MotorType::A1;
        cmd.mode  = queryMotorMode(MotorType::A1, MotorMode::FOC);
        cmd.id    = 0;
        cmd.kp    = kpRotorHip; //proportional or position term. i.e. stiffness
        cmd.kd    = kdRotorHip; //derivative or velocity term, i.e damping
        cmd.q     = hipRotorAngleDesired; //angle, radians
        cmd.dq    = 0.0; //angular velocity, radians/s
        cmd.tau   = hipTau; //rotor feedforward torque
        serial.sendRecv(&cmd, &data);
        torque = hipTau + kpRotorHip * (hipRotorAngleDesired - data.q) + kdRotorHip * (0.0 - data.dq);
        std::cout << std::endl;
        std::cout <<"Hip Motor" << std::endl;
        std::cout << "Angle (rad): " << data.q / gearRatio << std::endl;
        std::cout << "Angular Velocity (rad/s): " << data.dq / gearRatio << std::endl;
        std::cout << "Torque (N.m): " << torque << std::endl;
        std::cout << "Temperature: " << data.temp << std::endl;
        std::cout << "ISSUE? " << data.merror << std::endl;
        std::cout << std::endl;

        // Knee Motor Control
        //#####DETERMINING DESIRED ANGLE#######
        float kneeOutputAngleDesired;
        kneeOutputAngleDesired =  kneeAngleOutputInitial +  (thetaKnee)*sin(2*PI*sin_counter); //change thetaHip to 30
        float kneeRotorAngleDesired = (kneeOutputAngleDesired * (PI/180)) * gearRatio;

        cmd.motorType = MotorType::A1;
        data.motorType = MotorType::A1;
        cmd.mode  = queryMotorMode(MotorType::A1, MotorMode::FOC);
        cmd.id    = 1;
        cmd.kp    = kpRotorKnee; //proportional or position term. i.e. stiffness
        cmd.kd    = kdRotorKnee; //derivative or velocity term, i.e damping
        cmd.q     = kneeRotorAngleDesired; //angle, radians
        cmd.dq    = 0.0; //angular velocity, radians/s
        cmd.tau   = kneeTau; //rotor feedforward torque
        serial.sendRecv(&cmd, &data);
        torque = kneeTau + kpRotorKnee * (kneeRotorAngleDesired - data.q) + kdRotorKnee * (0.0 - data.dq);
        std::cout << std::endl;
        std::cout << "Knee Motor" << std::endl;
        std::cout << "Angle (rad): " << data.q / gearRatio << std::endl;
        std::cout << "Angular Velocity (rad/s): " << data.dq / gearRatio << std::endl;
        std::cout << "Torque (N.m): " << torque << std::endl;
        std::cout << "Temperature: " << data.temp << std::endl;
        std::cout << "ISSUE? " << data.merror << std::endl;
        std::cout << std::endl;

        // Wheel Motor Control
        //#####DETERMINING DESIRED ANGULAR VELOCITY#######
        cmd.motorType = MotorType::A1;
        data.motorType = MotorType::A1;
        cmd.mode  = queryMotorMode(MotorType::A1, MotorMode::FOC);
        cmd.id    = 2;
        cmd.kp    = 0.0; //proportional or position term. i.e. stiffness
        cmd.kd    = kdRotorWheel; //derivative or velocity term, i.e damping
        cmd.q     = 0.0; //angle, radians
        cmd.dq    = wheelRotorAngularVelocityDesired; //angular velocity, radians/s
        cmd.tau   = 0.0; //rotor feedforward torque
        serial.sendRecv(&cmd, &data);
        torque = 0.0 + 0.0 * (0.0 - data.q) + kdRotorWheel * (wheelRotorAngularVelocityDesired - data.dq);
        std::cout << std::endl;
        std::cout << "Wheel Motor" << std::endl;
        std::cout << "Angle (rad): " << data.q / gearRatio << std::endl;
        std::cout << "Angular Velocity (rad/s): " << data.dq / gearRatio << std::endl;
        std::cout << "Torque (N.m): " << torque << std::endl;
        std::cout << "Temperature: " << data.temp << std::endl;
        std::cout << "ISSUE? " << data.merror << std::endl;
        std::cout << std::endl;

        usleep(250);
    }
    //return 0;
}
