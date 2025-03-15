#include <iostream>
#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <math.h>

#define PI 3.1415926

int main() {

    // Global variables for serial communication
    SerialPort serial("/dev/ttyUSB0");
    MotorCmd cmd;
    MotorData data;

    float gear_ratio = queryGearRatio(MotorType::A1);

    // Initialize Hip Motor
    float kpHipOut = 25, kdHipOut = 0.6;
    auto [kpRotorHip, kdRotorHip] = outputToRotorCoefficients(kpHipOut, kdHipOut, gear_ratio);
    cmdActuator(0, 0.0, 0.0, 0.0, 0.0, 0.0);
    float hipAngleOutputInitial = getCurrentOutputAngle(gear_ratio);

    // Initialize Knee Motor
    float kpKneeOut = 25, kdKneeOut = 0.6;
    auto [kpRotorKnee, kdRotorKnee] = outputToRotorCoefficients(kpKneeOut, kdKneeOut, gear_ratio);
    cmdActuator(1, 0.0, 0.0, 0.0, 0.0, 0.0);
    float kneeAngleOutputInitial = getCurrentOutputAngle(gear_ratio);

    // Initialize Wheel Motor
    float kpWheelOut = 0.0, kdWheelOut = 0.6;
    auto [kpRotorWheel, kdRotorWheel] = outputToRotorCoefficients(kpWheelOut, kdWheelOut, gear_ratio);
    cmdActuator(2, 0.0, 0.0, 0.0, 0.0, 0.0);
    float wheelAngleOutputInitial = getCurrentOutputAngle(gear_ratio);

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
        outputData(0, torque, gear_ratio);

        // Knee Motor Control
        //#####DETERMINING DESIRED ANGLE#######
        cmdActuator(1, kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired, 0.0, kneeTau);
        torque = calculateOutputTorque(kpRotorKnee, kdRotorKnee, kneeRotorAngleDesired, 0.0, kneeTau, data.q, data.dq);
        outputData(1, torque, gear_ratio);

        // Wheel Motor Control
        //#####DETERMINING DESIRED ANGLE#######
        cmdActuator(2, 0.0, kdRotorWheel, 0.0, wheelRotorAngularVelocityDesired, 0.0);
        torque = calculateOutputTorque(0.0, kdRotorWheel, 0.0, wheelRotorAngularVelocityDesired, 0.0, data.q, data.dq);
        outputData(2, torque, gear_ratio);

        usleep(200);
    }

    return 0;
}

// Function to convert output gains to rotor gains
std::pair<float, float> outputToRotorCoefficients(float kpOutput, float kdOutput, float gear_ratio) {
    float kpRotor = (kpOutput / (gear_ratio * gear_ratio)) / 26.07;
    float kdRotor = (kdOutput / (gear_ratio * gear_ratio)) * 100.0;
    return std::make_pair(kpRotor, kdRotor);
}

// Function to get the current motor output angle
float getCurrentOutputAngle(float gear_ratio) {
    return (data.q / gear_ratio) * (180 / PI);
}

// Function to compute the desired rotor angle
float getDesiredRotorAngle(float outputAngleDesired, float gear_ratio) {
    return (outputAngleDesired * (PI / 180)) * gear_ratio;
}

// Function to send actuator commands
void cmdActuator(int id, float kp, float kd, float q, float dq, float tau) {
    cmd.motorType = MotorType::A1;
    data.motorType = MotorType::A1;
    cmd.mode  = queryMotorMode(MotorType::A1, MotorMode::FOC);
    cmd.id    = id;
    cmd.kp    = kp;
    cmd.kd    = kd;
    cmd.q     = q;
    cmd.dq    = dq;
    cmd.tau   = tau;

    serial.sendRecv(&cmd, &data);
}

// Function to compute output torque
float calculateOutputTorque(float kp, float kd, float qDesired, float dqDesired, float tau, float qCurrent, float dqCurrent) {
    return tau + kp * (qDesired - qCurrent) + kd * (dqDesired - dqCurrent);
}

// Function to output motor data
void outputData(int id, float torque, float gear_ratio) {
    std::string motorLabel = (id == 0) ? "Hip" : (id == 1) ? "Knee" : "Wheel";

    std::cout << std::endl;
    std::cout << motorLabel << " Motor" << std::endl;
    std::cout << "Angle (rad): " << data.q / gear_ratio << std::endl;
    std::cout << "Angular Velocity (rad/s): " << data.dq / gear_ratio << std::endl;
    std::cout << "Torque (N.m): " << torque << std::endl;
    std::cout << "Temperature: " << data.temp << std::endl;
    std::cout << "ISSUE? " << data.merror << std::endl;
    std::cout << std::endl;
}