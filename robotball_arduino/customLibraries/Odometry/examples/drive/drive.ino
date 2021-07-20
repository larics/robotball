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

unsigned long display_time = 200;
unsigned long last_display = 0;

unsigned long change_time = 5000;
unsigned long last_change = 0;
int current_speed = 0;
int direction = 1;


void setup()
{
	Serial.begin(115200);
	while (!Serial) yield();
}

void loop()
{
    odometry.update();

	unsigned long now = millis();

    if (now - last_change > change_time)
    {
        current_speed += 51;
        if (current_speed > 255)
        {
            current_speed = 0;
            direction -= 2;
        }

        motor_right.setSpeed(current_speed * direction);
        motor_left.setSpeed(-current_speed * direction);
        last_change = now;

        if (direction < -1)
        {
            Serial.print("========");
            Serial.println("  DONE  ");
            while(1);
        }
    }

	if (now - last_display > display_time)
	{
        float left, right;

        Serial.print("Speed: ");
        Serial.print(current_speed);
        Serial.print(" | ");
        
        odometry.getEncoderShift(&left, &right, true);
        Serial.print("Shift: ");
        Serial.print(left, 1);
        Serial.print(" ");
        Serial.print(right, 1);
        Serial.print(" | ");

        odometry.getWheelPos(&left, &right, true);
        Serial.print("Position: ");
        Serial.print(left, 1);
        Serial.print(" ");
        Serial.print(right, 1);
        Serial.print(" | ");

        odometry.getWheelOmega(&left, &right, true);
        Serial.print("Omega: ");
        Serial.print(left, 1);
        Serial.print(" ");
        Serial.print(right, 1);
        Serial.print(" | ");

        Serial.print("RPM: ");
        Serial.print(left * dps2rpm, 1);
        Serial.print(" ");
        Serial.print(right * dps2rpm, 1);
        Serial.print(" | ");

        Serial.println();

        last_display = now;
	}
}