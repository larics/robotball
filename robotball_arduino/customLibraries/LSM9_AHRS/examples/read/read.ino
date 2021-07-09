#include <Arduino.h>
#include "LSM9_AHRS.h"

double g_pitch;
double g_roll;
double g_hdg;

unsigned long display_time = 200;
unsigned long last_update = 0;

void setup()
{
	Serial.begin(115200);
	while (!Serial) yield();
	AHRS_setup();
}

void loop()
{
	AHRS_update(&g_roll, &g_pitch, &g_hdg);

	unsigned long now = millis();
	if (now - last_update > display_time)
	{
		Serial.print("HDG ");
		Serial.print(g_hdg);
		Serial.print(" ");
		Serial.print("PITCH ");
		Serial.print(g_pitch);
		Serial.print(" ");
		Serial.print("ROLL ");
		Serial.println(g_roll);

        last_update = now;
	}
}