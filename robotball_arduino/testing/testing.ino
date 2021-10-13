#include <CytronMotorDriver.h>
#include <PID_v2.h>
#include "LSM9_Mahony.h"
#include "Odometry.h"
#include "setup.h"
#include "utilities.h"


/************************ MOTORS-RELATED SETUP ********************************/
// Addapted from: https://github.com/CytronTechnologies/CytronMotorDriver
// Script name: PWM_PWM_DUAL.ino

// Configure the motor driver.
CytronMD motor_left(PWM_PWM, 5, 6);     // PWM 1A = Pin 5, PWM 1B = Pin 6.   << LEFT 
CytronMD motor_right(PWM_PWM, 9, 10);   // PWM 2A = Pin 9, PWM 2B = Pin 10.  << RIGHT 
/******************************************************************************/

/*********************** ODOMETRY-RELATED SETUP *******************************/
// Left  Encoder 1 = Pin 4, Left  Encoder 2 = Pin 7
// Right Encoder 1 = Pin 8, Right Encoder 2 = Pin 11
const EncoderPins pins{4, 7, 8, 11};
const DiffDriveParams params{
    .lwr = g_lwr,
    .rwr = g_rwr,
    .ws = g_ws,
    .sr = g_shell_radius,
    .cr = g_contact_radius
};
DiffDriveOdom odometry(pins, g_encoder_rate, params);
/******************************************************************************/

/************************* CONTROL-RELATED SETUP ******************************/
double g_speed_sp;
double g_pitch_sp;
double g_hdg_sp;

double g_speed;
double g_pitch;
double g_roll;
double g_hdg;

double g_vel_lin;
double g_vel_rot;

unsigned long last_cmd_vel;  // Last time velocity command was received

// v_ref -> pitch
PID PID_speed(&g_speed, &g_pitch_sp, &g_speed_sp, 0, 0, 0, 100, REVERSE);  // In, Out, Sp, Kp, Ki, Kd (2, 5, 1), Ts

// pitch -> motor_speed (linear.x)
PID PID_pitch(&g_pitch, &g_vel_lin, &g_pitch_sp, 0, 0, 0, 20, REVERSE); // In, Out, Sp, Kp, Ki, Kd (0.3/1.57, 0, 0), Ts

// hdg_ref -> motor_speed (angular.z)
PID PID_hdg(&g_hdg, &g_vel_rot, &g_hdg_sp, 0, 0, 0, 50, DIRECT);  // In, Out, Sp, Kp, Ki, Kd (2, 5, 1), Ts
/******************************************************************************/

/*************************** LOW-LEVEL STUFF **********************************/
const int8_t led_pin = 2;
const int8_t led_status = 1;

const int8_t bat_pin = A1;
/******************************************************************************/

void setup ()
{
	Serial.begin(115200);
	while (!Serial) yield();

	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(led_pin, OUTPUT);
	digitalWrite(led_pin, HIGH);

	// AHRS_setup();

	// Turn on or off individual PIDs.
	// Pitch must be enabled if speed is enabled.
  PID_pitch.SetMode(MANUAL);
	PID_speed.SetMode(MANUAL);
	PID_hdg.SetMode(MANUAL);

	PID_pitch.SetOutputLimits(-1, 1);
	PID_speed.SetOutputLimits(-g_pitch_scale, g_pitch_scale);
	PID_hdg.SetOutputLimits(-1, 1);

	PID_pitch.SetDeadzone(0);
	PID_hdg.SetDeadzone(0);
}


void loop() {
	double ignore;

	Serial.print("STRT! ");

	/* Read input on Serial */
	if (Serial.available() > 0) {
		read_and_set();
	}

	/* Get the latest orientation data. */
	// AHRS_update(&g_roll, &g_pitch, &g_hdg);

	/* Handle yaw wrapping */
	if (abs(g_hdg - g_hdg_sp) > PI)
	{
		if (g_hdg > 0) g_hdg_sp += 2 * PI;
		else g_hdg_sp -= 2 * PI;
	}

	/* Limit the pitch setpoint in large turns to avoid excessive rolling */
	// if (abs(g_hdg - g_hdg_sp) > PI / 3)
	// {
	// 	if (g_pitch_sp > PI / 12) {
	//   		g_pitch_sp = PI / 12;
	// 	}
	// }

	/* Get the latest odometry data. */
	odometry.update();
	odometry.getRobotVel(&g_speed, &ignore);

	/* Compute all PID outputs */
	PID_speed.Compute();
	PID_pitch.Compute();
	PID_hdg.Compute();

	debug("S_S", g_speed_sp);
	debug("S", g_speed);
	debug("H_S", g_hdg_sp);
	debug("H", g_hdg);
	debug("P_S", g_pitch_sp);
	debug("P", g_pitch);
	debug("R", g_roll);


	/* Control the motors. */
	double vel_left = 0;
	double vel_right = 0;

	vel_left = g_vel_lin - g_vel_rot;
	vel_right = g_vel_lin + g_vel_rot;

	debug("LIN", g_vel_lin);
	debug("ROT", g_vel_rot);

	double scale_factor = 1.0;

	if (abs(vel_left) > 1 || abs(vel_right) > 1)
	{
	  	double x = max(abs(vel_left), abs(vel_right));
	  	scale_factor = 1.0 / x;
	}

	vel_left = round(vel_left * scale_factor * 255);
	vel_right = round(vel_right * scale_factor * 255);

	// debug("vL", vel_left);
	// debug("vR", vel_right);
	debug("SPD", g_speed);


	Serial.println();

	// Set the motor speeds.
	motor_right.setSpeed(vel_right);
	motor_left.setSpeed(-vel_left);  // "-" because the motor is mounted differently
	/* --------------- */
}

double read_double(char ends) {
  double n = 0;
  int dec = 1;
  double d;
  bool pos = true;
  int8_t neg = 1;

  while (true) {
    if (Serial.available() > 0) {
      char c = (Serial.read());
      if (c == ends ) {
        return n * neg;
      }
      else if (c == '-') {
        neg = -1;
      }
      else if (c == '.') {
        pos = false;
      } else {
        d = c - '0';
        if (pos) {
          n = n * 10 + d;
        } else {
          dec *= 10;
          n += d / dec;
        }
      }
    }
  }
}

void read_and_set() {
	char c = (Serial.read());
	if (c == '<')
	{
		double Kp_speed, Ki_speed, Kd_speed;
		double Kp_pitch, Ki_pitch, Kd_pitch;
		double Kp_hdg, Ki_hdg, Kd_hdg;

		Kp_speed = read_double(';');
		Ki_speed = read_double(';');
		Kd_speed = read_double(';');
		Kp_pitch = read_double(';');
		Ki_pitch = read_double(';');
		Kd_pitch = read_double(';');
		Kp_hdg = read_double(';');
		Ki_hdg = read_double(';');
		Kd_hdg = read_double('>');

		PID_pitch.SetTunings(Kp_pitch, Ki_pitch, Kd_pitch);
		PID_speed.SetTunings(Kp_speed, Ki_speed, Kd_speed);
		PID_hdg.SetTunings(Kp_hdg, Ki_hdg, Kd_hdg);
	}
	else if (c == '[')
	{
		// Speed controller is active. Joystick commands the desired speed.
		if (PID_speed.GetMode())
		{
			g_speed_sp = read_double(';') * g_speed_scale;
			g_hdg_sp = read_double(']');
		}
		else if (PID_pitch.GetMode())
		{
			g_pitch_sp = read_double(';') * g_pitch_scale * -1;
			g_hdg_sp = read_double(']');
		}
		else if (PID_hdg.GetMode())
		{
			g_pitch_sp = read_double(';'); // Not really needed, just to move the serial buffer.
			g_hdg_sp = read_double(']');
		}
		else
		{
			g_vel_lin = read_double(';') * g_default_scale;
			g_vel_rot = read_double(']') / (PI);
		}
	}
}
