#ifndef _ROS_STUFF_H_
#define _ROS_STUFF_H_

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

// Global variables for messages/
//geometry_msgs::Pose pose;
std_msgs::Header info;

// Other global variables
float g_vel_lin;  	// X component of the commanded linear velocity
float g_vel_rot;	// Z component of the commanded angular velocity
int counter;    	// Counts the number of loops elapsed
unsigned long last_cmd_vel;	// Last time velocity command was received
const unsigned long cmd_vel_timeout = 2000;


// Callback functions
void cmd_vel_cb( const geometry_msgs::Twist& msg){
  // The latest received velocity setpoints are now stored in global variables.
  g_vel_lin = msg.linear.x;
  g_vel_rot = msg.angular.z;
  last_cmd_vel = millis();
}

#endif