#include <stdio.h>      // Standard input/output functions
#include <string.h>     // String handling functions
#include <unistd.h>     // UNIX system functions (usleep for sleep)
#include <stdlib.h>     // Standard library functions
#include "serialPort/SerialPort.h"  // Serial communication
#include "unitreeMotor/unitreeMotor.h"  // Unitree motor commands

#define BroadAllMotorID     0xBB  // Special ID to broadcast to all motors
#define MotorPulsator       11    // Mode to enter "pulsator mode" (for changing motor ID)

int main(){
    char serial_name[100];  // Stores the serial port name

    MotorCmd cmd;  // Stores motor command data
    MotorData data;  // Stores received motor data

    cmd.motorType = MotorType::B1;    // Set the motor type, either A1 or B1
    data.motorType = cmd.motorType;

    printf("Please input the name of the serial port. (e.g., Linux:/dev/ttyUSB0, Windows:\\\\.\\COM3)\n");
    scanf("%s",serial_name);
    printf("The serial port is %s\n", serial_name);

    SerialPort serial(serial_name);  // Set the serial port name

    cmd.id = BroadAllMotorID;  // Send command to all motors
    cmd.mode = 10;  // Set mode to 10 (possibly initialization or reset)
    serial.sendRecv(&cmd, &data);  // Send command and receive data

    usleep(100000);  // Sleep for 0.1 seconds

    // Enter pulsator mode (modify motor ID)
    cmd.mode = MotorPulsator;
    cmd.modify_data(&cmd);  // Modify the command data
    serial.send(cmd.get_cmdend_data(), cmd.hex_len);  // Send the modified command

    printf("Please turn the motor.\n");
    printf("One turn: id=0; Two turns: id=1, Three turns: id=2\n");
    printf("ID can only be 0, 1, or 2\n");
    printf("Once finished, press 'a'\n");

    while(getchar() != (int)'a');  // Wait for user to press 'a'
    printf("Turn finished\n");

    // Save the motor ID
    cmd.mode = 0;  // Set mode to 0 (possibly saving changes)
    cmd.modify_data(&cmd);  // Modify the command data
    serial.send(cmd.get_cmdend_data(), cmd.hex_len);  // Send the command

    return 0;
}
