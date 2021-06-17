#include "my_ahrs.h"
#include "motor.h"
#include <PID_v1.h>

#define ENABLE_SERIAL 1

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

// PID parameters
double Kp_speed;
double Ki_speed;
double Kd_speed;
double Kp_pitch;
double Ki_pitch;
double Kd_pitch;
double Kp_hdg;
double Ki_hdg;
double Kd_hdg;

// v_ref -> pitch
//PID PID_speed(&g_speed, &g_pitch_sp, &g_speed_sp, 2, 5, 1, DIRECT);  // In, Out, Sp, Kp, Ki, Kd (2, 5, 1)

// pitch -> motor_speed (linear.x)
PID PID_pitch(&g_pitch, &g_vel_lin, &g_pitch_sp, 0, 0, 0, DIRECT); // In, Out, Sp, Kp, Ki, Kd (0.3/1.57, 0, 0)

// hdg_ref -> motor_speed (angular.z)
PID PID_hdg(&g_hdg, &g_vel_rot, &g_hdg_sp, 0, 0, 0, DIRECT);  // In, Out, Sp, Kp, Ki, Kd (2, 5, 1)
/******************************************************************************/

void setup() {
  bool using_serial = AHRS_setup();
  //  if (ENABLE_SERIAL && !using_serial)
  //  {
  Serial.begin(115200);
  while (!Serial) yield();
  //}

  PID_pitch.SetMode(AUTOMATIC);
  //PID_speed.SetMode(AUTOMATIC);
  PID_hdg.SetMode(AUTOMATIC);

  PID_pitch.SetOutputLimits(-1, 1);
  PID_hdg.SetOutputLimits(-1, 1);

}


void loop() {
  //readin input on Serial
  if (Serial.available() > 0) {
    read_and_set();
  }
  /* Get the latest orientation data. */
  AHRS_update(&g_roll, &g_pitch, &g_hdg);
  //wrap to PI
  //  if (g_hdg_sp > PI || g_hdg_sp < -PI) {
  //    double x;
  //    x = g_hdg_sp + PI;
  //    while (g_hdg_sp >= 2*PI){
  //      g_hdg_sp -= 2*PI;
  //    }
  //    if (x > 0 && g_hdg_sp == 0) {
  //      g_hdg_sp = (2 * PI);
  //    }
  //    g_hdg_sp = g_hdg_sp - PI;
  //  }
  if (abs(g_hdg - g_hdg_sp) > PI) {
    if (g_hdg > 0) g_hdg_sp += 2 * PI;
    else g_hdg_sp -= 2 * PI;
  }

  if (abs(g_hdg - g_hdg_sp) > PI / 3) {
    if (g_pitch_sp > PI / 12) {
      g_pitch_sp = PI / 12;
    }
  }

  if (PID_pitch.Compute(0.1)) {
    //        Serial.print("PITCH ");
    //        Serial.print(g_pitch_sp);
    //        Serial.print(" ");
    //        Serial.print(g_pitch);
    //        Serial.print(" ");
    //        Serial.println(g_vel_lin);
  }
  //  PID_speed.Compute();
  if (PID_hdg.Compute(0.05)) {
    //    Serial.print("HDG ");
    //    Serial.print(g_hdg_sp);
    //    Serial.print(" ");
    //    Serial.print(g_hdg);
    //    Serial.print(" ");
    //    Serial.println(g_vel_rot);
  }
  Serial.print("HDG SP ");
  Serial.print(g_hdg_sp);
  Serial.print(" ");
  Serial.print("HDG ");
  Serial.print(g_hdg);
  Serial.print(" ");
  Serial.print("PITCH SP ");
  Serial.print(g_pitch_sp);
  Serial.print(" ");
  Serial.print("PITCH ");
  Serial.print(g_pitch);
  Serial.print(" ");
  Serial.print("ROLL ");
  Serial.print(g_roll);
  Serial.print(" ");

  /* Control the motors. */
  double vel_left = 0;
  double vel_right = 0;

  if ((millis() - last_cmd_vel <= cmd_vel_timeout) || true)
  {
    //    //     Calculate wheel speeds based on received velocity commands.
    //    vel_left  = round((g_vel_lin + g_vel_rot * g_ws / 2.0) / g_lwr * rad2pwm);
    //    vel_right = round((g_vel_lin - g_vel_rot * g_ws / 2.0) / g_rwr * rad2pwm);
    //
    //    // Clamp the resulting values.
    //    vel_left = constrain(vel_left, -255, 255);
    //    vel_right = constrain(vel_right, -255, 255);

    vel_left = g_vel_lin + g_vel_rot;
    vel_right = g_vel_lin - g_vel_rot;

        Serial.print(g_vel_lin);
        Serial.print(" ");
        Serial.print(g_vel_rot);
        Serial.print(" ");


    double scale_factor = 1;

    if (abs(vel_left) > 1 || abs(vel_right) > 1) {
      double x = max(abs(vel_left), abs(vel_right));
      scale_factor = 1.0 / x;
    }

    vel_left = round(vel_left * scale_factor * 255);
    vel_right = round(vel_right * scale_factor * 255);

        Serial.print(vel_left);
        Serial.print(" ");
        Serial.println(vel_right);

  }

  // Set the motor speeds.
  motor_right.setSpeed(-vel_right);
  motor_left.setSpeed(vel_left);  // "-" because the motor is mounted differently
  /* --------------- */
}


double read_double(char ends) {
  double n = 0;
  int dec = 1;
  double d;
  bool pos = true;
  int neg = 1;

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
  if (c == '<') {
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
    //PID_speed.SetTunings(Kp_speed, Ki_speed, Kd_speed);
    PID_hdg.SetTunings(Kp_hdg, Ki_hdg, Kd_hdg);
  }
  else if (c == '[') {
    g_pitch_sp = read_double(';');
    double new_hdg = read_double(']');
    if (new_hdg != 100) g_hdg_sp = new_hdg;

  }
  return;
}
