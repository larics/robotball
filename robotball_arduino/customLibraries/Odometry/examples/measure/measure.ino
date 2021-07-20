#include <Arduino.h>
#include <CytronMotorDriver.h>
#include "Odometry.h"

/************************ MOTORS-RELATED SETUP ********************************/
// Addapted from: https://github.com/CytronTechnologies/CytronMotorDriver
// Script name: PWM_PWM_DUAL.ino

// Configure the motor driver.
CytronMD motor_left(PWM_PWM, 5, 9);     // PWM 1A = Pin 5, PWM 1B = Pin 9.   << LEFT 
CytronMD motor_right(PWM_PWM, 10, 11);  // PWM 2A = Pin 10, PWM 2B = Pin 11. << RIGHT
const float dps2rpm = 0.166667;
/******************************************************************************/

/*********************** ODOMETRY-RELATED SETUP *******************************/
// Left  Encoder 1 = Pin 2, Left  Encoder 1 = Pin 6
// Right Encoder 1 = Pin 3, Right Encoder 1 = Pin 7
EncoderPins pins{2, 6, 3, 7};
DiffDriveParams params{
    .lwr = 0.03875,
    .rwr = 0.03875,
    .ws = 0.323
};
int g_encoder_rate = 1680;
DiffDriveOdom odometry(pins, g_encoder_rate, params);
/******************************************************************************/

unsigned long display_time = 500;
unsigned long last_display = 0;

unsigned long loop_time = 100;
unsigned long last_loop = 0;
int current_speed = 0;

double left_average = 0;
long left_size = 0;

double right_average = 0;
long right_size = 0;


void setup()
{
	Serial.begin(115200);
	while (!Serial) yield();

    motor_right.setSpeed(127);
}

void loop()
{
    odometry.update();

	unsigned long now = millis();

    if (now - last_loop > loop_time)
    {
        float left, right;
        odometry.getWheelOmega(&left, &right, true);
        left *= 0.166667;
        right *= 0.166667;

        left_average = (left_size * left_average + left) / (left_size + 1);
        left_size++;

        right_average = (right_size * right_average + right) / (right_size + 1);
        right_size++;

        last_loop = now;

        if (now - last_display > display_time)
        {
            Serial.print("Time: ");
            Serial.print(round(now / 1000));
            Serial.print(" | ");

            Serial.print("Current: ");
            Serial.print(left, 1);
            Serial.print(" ");
            Serial.print(right, 1);
            Serial.print(" | ");

            Serial.print("Average: ");
            Serial.print(left_average, 1);
            Serial.print(" ");
            Serial.print(right_average, 1);
            Serial.println();

            last_display = now;
        }
    }
}