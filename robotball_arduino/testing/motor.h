#ifndef _MOTOR_H
#define _MOTOR_H

#include <CytronMotorDriver.h>

const float rpm2rad = 0.104719755;
const float max_motor_rpm = 75;
const float rad2pwm = 255 / (max_motor_rpm * rpm2rad);

#endif
