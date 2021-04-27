#include "main.h"

/*************************** ROS-RELATED SETUP ********************************/

// The handle for ROS node running on Arduino
ros::NodeHandle_<ArduinoHardware, 2, 2, 280, 280> nh;

// Publishers
//ros::Publisher odomPub("odom", &pose);
ros::Publisher infoPub("info", &info);

// Subscribers
ros::Subscriber<geometry_msgs::Twist> cmdSub("cmd_vel", cmd_vel_cb );

/******************************************************************************/


/************************ MOTORS-RELATED SETUP ********************************/

// Addapted from: https://github.com/CytronTechnologies/CytronMotorDriver
// Script name: PWM_PWM_DUAL.ino

// Configure the motor driver.
CytronMD motor1(PWM_PWM, 5, 9);   // PWM 1A = Pin 5, PWM 1B = Pin 9.   << LEFT 
CytronMD motor2(PWM_PWM, 10, 11); // PWM 2A = Pin 10, PWM 2B = Pin 11. << RIGHT 

/******************************************************************************/

void setup()
{
  // Set the baudrate and initialize the node.
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  // Initialize publishers and subscribers.
  nh.advertise(infoPub);
  nh.subscribe(cmdSub);

  /**** Encoders are still work in progress. ****/
  // attachInterrupt(digitalPinToInterrupt(ENCODER1), encoder, RISING);

  // pinMode(ENCODER1, INPUT);
  // pinMode(ENCODER2, INPUT);

  // pos_pid.begin();    
  // pos_pid.tune(15, 0, 2000);    
  // pos_pid.limit(-255, 255);
  // pos_pid.setpoint(0);

  // speed_pid.begin();   
  // speed_pid.tune(10, 0, 100);    
  // speed_pid.limit(-255, 255);
  // speed_pid.setpoint(0); 
  
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

  // Calculate wheel speeds based on received velocity commands.
  int vel_left  = round((g_vel_lin - g_vel_rot * g_ws / 2.0) / g_lwr * rad2pwm);
  int vel_right = round((g_vel_lin + g_vel_rot * g_ws / 2.0) / g_rwr * rad2pwm);

  // Clamp the resulting values.
  vel_left = constrain(vel_left, -255, 255);
  vel_right = constrain(vel_right, -255, 255);
  
  // Set the motor speeds.
  motor1.setSpeed(vel_right);
  motor2.setSpeed(-vel_left);  // "-" because the motor is mounted differently
  
  delay(100); // Update frequency is 10 Hz. 
}


// void encoder(){

//   if(digitalRead(ENCODER2) == HIGH){
//     encoder_pos++;
//   }else{
//     encoder_pos--;
//   }
// }