#include "my_ahrs.h"
#include "motor.h"
#include <PID_v1.h>

#define ENABLE_SERIAL 0

/************************ MOTORS-RELATED SETUP ********************************/
// Addapted from: https://github.com/CytronTechnologies/CytronMotorDriver
// Script name: PWM_PWM_DUAL.ino

// Configure the motor driver.
CytronMD motor_left(PWM_PWM, 5, 9);     // PWM 1A = Pin 5, PWM 1B = Pin 9.   << LEFT 
CytronMD motor_right(PWM_PWM, 10, 11);  // PWM 2A = Pin 10, PWM 2B = Pin 11. << RIGHT 
/******************************************************************************/


/*********************** ENCODERS-RELATED SETUP *******************************/
// Configure the encoders.
Encoder enc_left(2, 6);   // Left  Encoder 1 = Pin 2, Left  Encoder 1 = Pin 6
Encoder enc_right(3, 7);  // Right Encoder 1 = Pin 3, Right Encoder 1 = Pin 7
/******************************************************************************/


/*********************** CONTROL-RELATED SETUP *******************************/
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
const unsigned long cmd_vel_timeout = 2000;

// Geometry configuration
const float g_ws = 0.323;    // Wheel separation (m) == 32.3 cm
const float g_lwr = 0.03875;  // Left wheel radius (m) == 38.75 mm
const float g_rwr = 0.03875;  // Right wheel radius (m) == 38.75 mm

// v_ref -> pitch
PID PID_speed(&g_speed, &g_pitch_sp, &g_speed_sp, 2, 5, 1, DIRECT);  // In, Out, Sp, Kp, Ki, Kd

// pitch -> motor_speed (linear.x)
PID PID_pitch(&g_pitch, &g_vel_lin, &g_pitch_sp, 0.3/1.57, 0, 0, DIRECT);  // In, Out, Sp, Kp, Ki, Kd

// hdg_ref -> motor_speed (angular.z)
PID PID_hdg(&g_hdg, &g_vel_rot, &g_hdg_sp, 2, 5, 1, DIRECT);  // In, Out, Sp, Kp, Ki, Kd
/******************************************************************************/

void setup() {
  bool using_serial = AHRS_setup();
  if (ENABLE_SERIAL && !using_serial)
  {
    Serial.begin(115200);
    while (!Serial) yield();
  }

  PID_pitch.SetMode(AUTOMATIC);
  PID_pitch.SetOutputLimits(-0.3, 0.3);
}


void loop() {
  /* Get the latest orientation data. */
  AHRS_update(&g_roll, &g_pitch, &g_hdg);

  PID_pitch.Compute();


  /* Control the motors. */
  int vel_left = 0;
  int vel_right = 0;
  
  if ((millis() - last_cmd_vel <= cmd_vel_timeout) || true)
  {
    // Calculate wheel speeds based on received velocity commands.
    vel_left  = round((g_vel_lin + g_vel_rot * g_ws / 2.0) / g_lwr * rad2pwm);
    vel_right = round((g_vel_lin - g_vel_rot * g_ws / 2.0) / g_rwr * rad2pwm);

    // Clamp the resulting values.
    vel_left = constrain(vel_left, -255, 255);
    vel_right = constrain(vel_right, -255, 255);
    Serial.print(vel_left);
    Serial.print("  ");
    Serial.println(vel_right);
  }
  
  // Set the motor speeds.
  motor_right.setSpeed(-vel_right);
  motor_left.setSpeed(vel_left);  // "-" because the motor is mounted differently
  /* --------------- */
}
