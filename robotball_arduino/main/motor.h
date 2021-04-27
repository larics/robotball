#ifndef _MOTOR_H
#define _MOTOR_H

#include "utilities.h"
#include "CytronMotorDriver.h"
#include <PIDController.h> 

const float max_motor_rpm = 75;
const float rad2pwm = 255 / (max_motor_rpm * rpm2rad);


// volatile long int encoder_pos = 0;
// PIDController pos_pid; 
// PIDController speed_pid; 
// int motor_mode = 0; // 0 - none, 1 - position control, 2 speed control
// long int last_encoder_pos = 0;
// long int last_millis = 0;
// int motor_value = 255;

// String inputString = "";         // a String to hold incoming data
// bool stringComplete = false;  // whether the string is complete


// #define ENCODER1 2
// #define ENCODER2 4   // <--- this was 3, check if works. 
#endif