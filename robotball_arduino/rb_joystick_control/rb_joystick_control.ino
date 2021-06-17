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

double vel_left;
double vel_right;

unsigned long last_cmd_vel;  // Last time velocity command was received
const unsigned long cmd_vel_timeout = 2000;

// Geometry configuration
const float g_ws = 0.323;    // Wheel separation (m) == 32.3 cm
const float g_lwr = 0.03875;  // Left wheel radius (m) == 38.75 mm
const float g_rwr = 0.03875;  // Right wheel radius (m) == 38.75 mm

/******************************************************************************/

void setup() {
  bool using_serial = AHRS_setup();
    Serial.begin(115200);
    while (!Serial) yield();



}


void loop() {
  if (Serial.available()>0){
    read_vels(&vel_left, &vel_right);
  }
    /* Get the latest orientation data. */
  AHRS_update(&g_roll, &g_pitch, &g_hdg);


  Serial.print(vel_left);
  Serial.print(" ");
  Serial.println(vel_right);
  /* Control the motors. */
  

    motor_right.setSpeed(vel_right);
    motor_left.setSpeed(vel_left);  // "-" because the motor is mounted differently

  
  
  /* --------------- */
}


double read_double(){
    double n = 0;
    int dec = 1;
    double d;
    bool pos = true;
    int neg = 1;

    while (true){
      if (Serial.available() > 0){
        char c = (Serial.read());
        if (c == ';' || c == ']'){
            return n*neg;
        }
        else if (c == '-'){
            neg = -1;
        }
        else if (c == '.'){
          pos = false;
        }else{
          d = c - '0';
          if (pos) {
            n= n*10 + d;
          } else{
            dec *= 10;
            n += d/dec;
          }
       }
      }
    }
}

void read_vels(double *vel_left, double *vel_right){
        char c = (Serial.read());
        if (c == '['){
          *vel_left = read_double();
          *vel_right = read_double();
        }
        return;
}
