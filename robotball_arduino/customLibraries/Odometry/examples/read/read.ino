#include <Arduino.h>
#include "Odometry.h"


EncoderPins pins{2, 6, 3, 7};
DiffDriveParams params{
    .lwr = 0.03875,
    .rwr = 0.03875,
    .ws = 0.323,
    .sr = 0.1775,
    .cr = 0.0666
};
int g_encoder_rate = 1680;

DiffDriveOdom odometry(pins, g_encoder_rate, params);

unsigned long display_time = 200;
unsigned long last_update = 0;

void setup()
{
	Serial.begin(115200);
	while (!Serial) yield();
}

void loop()
{

	odometry.update();

	unsigned long now = millis();
	if (now - last_update > display_time)
	{
        float left, right;
        
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

        Serial.println();

        last_update = now;
	}
}