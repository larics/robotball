#ifndef _UTILS_H_
#define _UTILS_H_


void debug(char* title, int value)
{
	Serial.print(title);
	Serial.print(": ");
	Serial.print(value);
	Serial.print(" | ");
}

void debug(char* title, float value)
{
	Serial.print(title);
	Serial.print(": ");
	Serial.print(value, 3);
	Serial.print(" | ");
}

void debug(char* title, double value)
{
	Serial.print(title);
	Serial.print(": ");
	Serial.print(value, 3);
	Serial.print(" | ");
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


#endif