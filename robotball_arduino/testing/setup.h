#ifndef _SETUP_H_
#define _SETUP_H_

const float g_lwr = 0.045;  			// Left wheel radius (m) == 45 mm
const float g_rwr = 0.045;  			// Right wheel radius (m) == 45 mm
const float g_ws  = 0.320;				// Wheel separation (m) == 32.0 cm
const float g_shell_radius = 0.1765;	// Outer shell radius (m) == 17.65 cm
const float g_contact_radius = 0.0860;	// Radius of contact point between wheels and inner shell (m) == 8.60 cm

const float g_k1 = 1;  // Odometry calibration coefficient 1
const float g_k2 = 1;  // Odometry calibration coefficient 2
const float g_k3 = 1;  // Odometry calibration coefficient 3

const float g_max_motor_rpm = 75;
const int g_encoder_rate = 1680;	// Number of signals per rotation.
									// Encoder's resolution is 420, but we use 4x counting.

const int8_t l_motor_sign = 1;	// Set to -1 if motor is mounted in reverse.
const int8_t r_motor_sign = -1;


const float g_pitch_scale = PI / 4;
const float g_speed_scale = g_max_motor_rpm * TWO_PI / 60 * ((g_lwr + g_rwr) / 2) * (g_shell_radius / g_contact_radius);
// 0.724984012

const unsigned long cmd_vel_timeout = 2000;	// Stop if no command is received after this many milliseconds.

#endif
