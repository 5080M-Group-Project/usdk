#include <iostream>
#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <math.h>

#define PI 3.1415926

SerialPort serial("/dev/ttyUSB0");
MotorCmd cmd;
MotorData data;
float gearRatio = queryGearRatio(MotorType::A1);

// Functions to convert output gains to rotor gains
float kpOutputToRotor(float kpOutput){
    return (kpOutput / (gearRatio * gearRatio)) / 26.07;
}

float kdOutputToRotor(float kdOutput){
    return (kdOutput / (gearRatio * gearRatio)) / 100.0;
}

// Function to get the current motor output angle in DEGREES
float getOutputAngle(float rotorAngle) {
    return (rotorAngle / gearRatio) * (180 / PI);
}

// Function to compute the desired rotor angle in RADIANS
float getRotorAngle(float outputAngle) {
    return (outputAngle * (PI / 180)) * gearRatio;
}

// Function to send actuator commands
void cmdActuator(int id, float kp, float kd, float q, float dq, float tau) {
    cmd.motorType = MotorType::A1;
    data.motorType = MotorType::A1;
    cmd.mode  = queryMotorMode(MotorType::A1, MotorMode::FOC);
    cmd.id    = id;
    cmd.kp    = kp; //proportional or position term. i.e. stiffness
    cmd.kd    = kd; //derivative or velocity term, i.e damping
    cmd.q     = q; //angle, radians
    cmd.dq    = dq; //angular velocity, radians/s
    cmd.tau   = tau; //rotor feedforward torque

    serial.sendRecv(&cmd, &data);
}

// Function to compute output torque
float calculateOutputTorque(float kp, float kd, float qDesired, float dqDesired, float tau, float qCurrent, float dqCurrent) {
    return tau + kp * (qDesired - qCurrent) + kd * (dqDesired - dqCurrent);
}

// Function to output motor data
void outputData(int id, float q, float dq, float torque, float temp, bool merror) {

    std::cout << std::endl;
    std::string motorLabel = (id == 0) ? "Hip" : (id == 1) ? "Knee" : "Wheel";
    std::cout << motorLabel << " Motor" << std::endl;
    std::cout << "Angle (rad): " << q / gearRatio << std::endl;
    std::cout << "Angular Velocity (rad/s): " << dq / gearRatio << std::endl;
    std::cout << "Torque (N.m): " << torque << std::endl;
    std::cout << "Temperature: " << temp << std::endl;
    std::cout << "ISSUE? " << merror << std::endl;
    std::cout << std::endl;
}


int main() {

    SerialPort serial("/dev/ttyUSB0");
    MotorCmd cmd;
    MotorData data;

    // Initialize Hip Motor
    float kpOutHip = 25, kdOutHip = 0.6, kpRotorHip = 0.0, kdRotorHip = 0.0;
    kpRotorHip = kpOutputToRotor(kpOutHip);
    kdRotorHip = kdOutputToRotor(kdOutHip);
    //          id  kp   kd   q   dq   tau
    cmdActuator(0, 0.0, 0.0, 0.0, 0.0, 0.0);
    float hipAngleOutputInitial = getOutputAngle(data.q);

    // Initialize Knee Motor
    float kpOutKnee = 25, kdOutKnee = 0.6, kpRotorKnee = 0.0, kdRotorKnee = 0.0;
    kpRotorKnee = kpOutputToRotor(kpOutKnee);
    kdRotorKnee = kdOutputToRotor(kdOutKnee);
    cmdActuator(1, 0.0, 0.0, 0.0, 0.0, 0.0);
    float kneeAngleOutputInitial = getOutputAngle(data.q);

    // Initialize Wheel Motor
    float kpOutWheel = 0.0, kdOutWheel = 2.0, kpRotorWheel = 0.0, kdRotorWheel = 0.0;
    kpRotorWheel = kpOutputToRotor(kpOutWheel);
    kdRotorWheel = kdOutputToRotor(kdOutWheel);
    cmdActuator(2, 0.0, 0.0, 0.0, 0.0, 0.0);
    float wheelAngleOutputInitial = getOutputAngle(data.q);

    //Initialize Loop Variables
    float torque = 0.0;
    float hipRotorAngleDesired = 0.0;
    float kneeRotorAngleDesired = 0.0;
    float wheelRotorAngularVelocityDesired = 0.0;
    float hipTau = 0.0;
    float kneeTau = 0.0;

    while (true) {
        // Hip Motor Control
        //#####DETERMINING DESIRED ANGLE#######
        cmdActuator(0, kpRotorHip, kdRotorHip, hipRotorAngleDesired, 0.0, hipTau);
        torque = calculateOutputTorque(kpRotorHip, kdRotorHip, hipRotorAngleDesired, 0.0, hipTau, data.q, data.dq);
        outputData(0, data.q, data.dq, torque, data.temp, data.merror);

        // Knee Motor Control
        //#####DETERMINING DESIRED ANGLE#######
        cmdActuator(1, kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired, 0.0, kneeTau);
        torque = calculateOutputTorque(kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired, 0.0, kneeTau, data.q, data.dq);
        outputData(1, data.q, data.dq, torque, data.temp, data.merror);

        // Wheel Motor Control
        //#####DETERMINING DESIRED ANGULAR VELOCITY#######
        cmdActuator(2, 0.0, kdRotorWheel, 0.0, wheelRotorAngularVelocityDesired, 0.0);
        torque = calculateOutputTorque(0.0, kdRotorWheel, 0.0, wheelRotorAngularVelocityDesired, 0.0, data.q, data.dq);
        outputData(2, data.q, data.dq, torque, data.temp, data.merror);

        usleep(200);
    }
    return 0;
}
