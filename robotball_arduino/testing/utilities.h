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


#endif