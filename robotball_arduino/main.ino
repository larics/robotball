/////////////////////////////////
///////// ROS RELATED ///////////

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "CytronMotorDriver.h"
#include <PIDController.h>

// The handle for ROS node running on Arduino
ros::NodeHandle_<ArduinoHardware, 2, 2, 280, 280> nh;

// Global variables for messages/
//geometry_msgs::Pose pose;
std_msgs::Header info;

// Other global variables
float g_vel_x;  // X component of the commanded velocity
float g_vel_y;  // Y component of the commanded velocity
float g_vel_z;
int counter;    // Counts the number of loops elapsed

// Publishers
//ros::Publisher odomPub("odom", &pose);
ros::Publisher infoPub("info", &info);


// Subscribers and callback functions
void cmd_vel_cb( const geometry_msgs::Twist& msg){
  // The latest received velocity setpoints are now stored in global variables.
  g_vel_x = msg.linear.x;
  g_vel_y = msg.linear.y;
  g_vel_z = msg.angular.z;
}
ros::Subscriber<geometry_msgs::Twist> cmdSub("cmd_vel", cmd_vel_cb );

///////////////////////////////////
///////// MOTOR RELATED ///////////

/*******************************************************************************
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTY AND SUPPORT
 * IS APPLICABLE TO THIS SOFTWARE IN ANY FORM. CYTRON TECHNOLOGIES SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 ********************************************************************************
 * DESCRIPTION:
 *
 * This example shows how to drive 2 motors using 4 PWM pins (2 for each motor)
 * with 2-channel motor driver.
 * 
 * 
 * CONNECTIONS:
 * 
 * Arduino D3  - Motor Driver PWM 1A Input
 * Arduino D9  - Motor Driver PWM 1B Input
 * Arduino D10 - Motor Driver PWM 2A Input
 * Arduino D11 - Motor Driver PWM 2B Input
 * Arduino GND - Motor Driver GND
 *
 *
 * AUTHOR   : Kong Wai Weng
 * COMPANY  : Cytron Technologies Sdn Bhd
 * WEBSITE  : www.cytron.io
 * EMAIL    : support@cytron.io
 *
 *******************************************************************************/

volatile long int encoder_pos = 0;
PIDController pos_pid; 
PIDController speed_pid; 
int motor_mode = 0; // 0 - none, 1 - position control, 2 speed control
long int last_encoder_pos = 0;
long int last_millis = 0;
int motor_value = 255;

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete


#define ENCODER1 2
#define ENCODER2 4   // <--- this was 3, check if works. 

// Configure the motor driver.
CytronMD motor1(PWM_PWM, 5, 9);   // PWM 1A = Pin 3, PWM 1B = Pin 9.                  << LEFT MOTOR
CytronMD motor2(PWM_PWM, 10, 11); // PWM 2A = Pin 10, PWM 2B = Pin 11.                << RIGHT MOTOR 





void setup()
{
//  pinMode(5, OUTPUT);              <<<- redundant, now defined in CytronMD  (3,9) and (10,11)
//  pinMode(6, OUTPUT);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(infoPub);
  nh.subscribe(cmdSub);


  //attachInterrupt(digitalPinToInterrupt(ENCODER1), encoder, RISING);

  //pinMode(ENCODER1, INPUT);
  //pinMode(ENCODER2, INPUT);

  pos_pid.begin();    
  pos_pid.tune(15, 0, 2000);    
  pos_pid.limit(-255, 255);
  pos_pid.setpoint(0);

  speed_pid.begin();   
  speed_pid.tune(10, 0, 100);    
  speed_pid.limit(-255, 255);
  speed_pid.setpoint(0); 
  
}

void loop()
{
  counter++;

  // Incoming messages are stored in a buffer.
  // This function calls the appropriate callback functions to process the data.
  nh.spinOnce();  
  
  // Prepare basic info message and publish.
  info.seq = counter;
  infoPub.publish( &info );

  // Do stuff with received commanded velocity.
  int x = g_vel_x * 255;
  int y = g_vel_y * 255;
  int rotation = g_vel_z*255;

  int speed_right = x;
  int speed_left = x;
  if(rotation != 0){
    if (rotation >= 0){

      speed_left = speed_left+ rotation;
      speed_right = speed_right -rotation;

    }
    else if (rotation <= 0){

      speed_left = speed_left - rotation;
      speed_right = speed_right + rotation;

    }    
    
 }
  
  motor1.setSpeed(speed_right);        // replaces analogWrite(5, x);      
  motor2.setSpeed(-speed_left);         // replaces analogWrite(6, y);

  delay(100);                // <---- why this delay? To decide the updates / second? 
 
 // motor1.setSpeed(0); 
}





void encoder(){

  if(digitalRead(ENCODER2) == HIGH){
    encoder_pos++;
  }else{
    encoder_pos--;
  }
}

void MotorClockwise(int power){
   motor1.setSpeed(power); 
  Serial.println("CW"); 
}