#ifndef _SETUP_H_
#define _SETUP_H_

const float g_lwr = 0.03875;  			// Left wheel radius (m) == 38.75 mm
const float g_rwr = 0.03875;  			// Right wheel radius (m) == 38.75 mm
const float g_ws  = 0.323;				// Wheel separation (m) == 32.3 cm
const float g_shell_radius = 0.1775;	// Outer shell radius (m) == 17.75 cm
const float g_contact_radius = 0.0666;	// Radius of contact point between wheels and inner shell (m) == 6.66 cm

const float g_max_motor_rpm = 75;
const int g_encoder_rate = 1680;	// Number of signals per rotation.
									// Encoder's resolution is 420, but we use 4x counting.

const int8_t l_motor_sign = 1;	// Set to -1 if motor is mounted in reverse.
const int8_t r_motor_sign = -1;


const float g_default_scale = 1;
const float g_pitch_scale = PI / 4;
const float g_speed_scale = g_max_motor_rpm * TWO_PI / 60 * ((g_lwr + g_rwr) / 2) * (g_shell_radius / g_contact_radius);


const unsigned long cmd_vel_timeout = 2000;	// Stop if no command is received after this many milliseconds.

#endif