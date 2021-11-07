#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <robotball_msgs/Odometry.h>
#include <robotball_msgs/Status.h>
#include <robotball_msgs/Debug.h>
#include <robotball_msgs/IMU.h>
#include <robotball_msgs/DynReconf.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <arduino-timer.h>
#include <CytronMotorDriver.h>
#include <math.h>

#include <PID_v2.h>
#include <Odometry.h>
#include <RobotUtilities.h>

#include "setup.h"
#include "utilities.h"


/*************************** MOTORS-RELATED SETUP *****************************/
// Addapted from: https://github.com/CytronTechnologies/CytronMotorDriver
// Script name: PWM_PWM_DUAL.ino

// Configure the motor driver.
CytronMD motor_left( PWM_PWM, 9, 10);   // PWM 1A = Pin 9, PWM 1B = Pin 10. << LEFT
CytronMD motor_right(PWM_PWM, 5, 6 );   // PWM 2A = Pin 5, PWM 2B = Pin 6.  << RIGHT
/******************************************************************************/

/************************** ODOMETRY-RELATED SETUP ****************************/
// Left  Encoder 1 = Pin 4, Left  Encoder 2 = Pin 7
// Right Encoder 1 = Pin 8, Right Encoder 2 = Pin 11
const EncoderPins pins{8, 11, 4, 7};
const DiffDriveParams params{
    .lwr = g_lwr * l_motor_sign,
    .rwr = g_rwr * r_motor_sign,
    .ws = g_ws,
    .sr = g_shell_radius,
    .cr = g_contact_radius,
    .k1 = g_k1,
    .k2 = g_k2,
    .k3 = g_k3
};
DiffDriveOdom odometry(pins, g_encoder_rate, params);
/******************************************************************************/

/**************************** IMU-RELATED SETUP *******************************/
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
const imu::Quaternion rot_offset(0.5, -0.5, 0.5, 0.5);
/******************************************************************************/

/************************* CONTROL-RELATED SETUP ******************************/
double g_speed;
double g_pitch;
double g_roll;
double g_hdg;

double g_vel_lin;
double g_vel_rot;

bool g_cal_lin = false;
bool g_cal_rot = false;
/******************************************************************************/

/*************************** LOW-LEVEL STUFF **********************************/
const int8_t led_pin = 2;
const int8_t led_status = 1;
/******************************************************************************/

/******************************* ROS SETUP ************************************/
// ros::NodeHandle_<ArduinoHardware, 5, 3, 256, 256> nh;
ros::NodeHandle nh;

robotball_msgs::Status status_msg;
ros::Publisher status_pub("status", &status_msg);

robotball_msgs::Debug debug_msg;
ros::Publisher debug_pub("debug", &debug_msg);

void cmdVelCb (const geometry_msgs::Twist& cmd_vel) {
	if (cmd_vel.linear.x == 1)
	    g_cal_lin = true;
	else if (cmd_vel.linear.y == 1)
	    g_cal_rot = true;
}
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &cmdVelCb);
}
/******************************************************************************/

void setup ()
{
    /* Set up pins. */
    pinMode(LED_BUILTIN, OUTPUT);
	pinMode(led_pin, OUTPUT);

	/* Set up ROS. */
    nh.getHardware()->setBaud(115200);
	nh.initNode();
    nh.advertise(debug_pub);
    nh.advertise(status_pub);
    nh.subscribe(cmd_sub);
    nh.spinOnce();

	/* Initialize BNO055 IMU. */
	while(!bno.begin())
	{
		// There was a problem detecting the BNO055.
		for(int i=0; i<5; i++) {
			digitalWrite(led_pin, HIGH);
			delay(500);
			digitalWrite(led_pin, LOW);
			delay(500);
			status_msg.calibration = 10000;
			status_pub.publish(&status_msg);
			nh.spinOnce();
		}
	}
	delay(1000);
	bno.setExtCrystalUse(true);

	/* Check the calibration status of the IMU. */
	uint8_t system, gyro, accel, mag;
	system = gyro = accel = mag = 0;
	// The data should be ignored until the system calibration is > 0.
	int valid_count = 0;
	while (valid_count < 20)
	{
		bno.getCalibration(&system, &gyro, &accel, &mag);
		valid_count = valid_count * (system > 0) + 1;
		status_msg.calibration = 10000 + system * 1000 + gyro * 100 + accel * 10 + mag;
		status_pub.publish(&status_msg);
		nh.spinOnce();
		delay(50);
	}
	status_msg.calibration = 90000 + system * 1000 + gyro * 100 + accel * 10 + mag;
	status_pub.publish(&status_msg);
	nh.spinOnce();
	digitalWrite(led_pin, HIGH);
	nh.loginfo("IMU calibrated. Ready for odometry calibration.");

	// Send robot's parameters.
	for(int i=0; i<10; i++)
	{
        debug_msg.odom.pos.left = g_lwr;
        debug_msg.odom.pos.right = g_rwr;
        debug_msg.odom.omega.left = g_ws;
        debug_msg.odom.omega.right = g_shell_radius / g_contact_radius;
        debug_pub.publish(&debug_msg);
        nh.spinOnce();
	    delay(50)
	}

	linear_calibration();
    odometry.reset();
	rotation_calibration();
}


void loop() {}


void linear_calibration()
{
    while (!g_cal_lin)
	{
        nh.spinOnce();
        delay(50);
	}
	int i = 0;
	int speed = 0;
    while (i < 200)
    {
        if (i < 50) speed += 2;
        if (i >= 150) speed -= 2;
        motor_right.setSpeed(r_motor_sign * 150);
	    motor_left.setSpeed(l_motor_sign * -150);

	    if (odometry.update())
        {
            imu::Quaternion quat = bno.getQuat();
            quat = quat * rot_offset;
            imu::Vector<3> quat_euler = quat.toEuler();
            double hdg = RobotUtils::wrap_0_2pi(quat_euler.x());

            float wl, wr, dt;
            float x, y, theta;
            odometry.getRobotPos(&x, &y, &theta, false)
            odometry.getWheelOmega(&wl, &wr, false);
            odometry.getTimeDelta(&dt);
            debug_msg.odom.pos.left = x;
            debug_msg.odom.pos.right = y;
            debug_msg.hdg.measured = theta;
            debug_msg.hdg.setpoint = hdg;
            debug_msg.odom.omega.left = wl;
	        debug_msg.odom.omega.right = wr;
	        debug_msg.odom.time = dt;
	        debug_pub.publish(&debug_msg);
	        nh.spinOnce();
        }
    }
    motor_right.setSpeed(0);
    motor_left.setSpeed(0);
    nh.loginfo("Linear calibration completed.");
}


void rotation_calibration()
{
    while (!g_cal_rot)
	{
        nh.spinOnce();
        delay(50);
	}
	imu::Quaternion quat = bno.getQuat();
    quat = quat * rot_offset;
    imu::Vector<3> quat_euler = quat.toEuler();
    double last_hdg = RobotUtils::wrap_0_2pi(quat_euler.x());
    double total_rot = 0;
    motor_right.setSpeed(r_motor_sign * 150);
	motor_left.setSpeed(l_motor_sign * -150);
    while (total_rot < 20 * 2 * PI)
    {
        /* Get the latest orientation data. */
        imu::Quaternion quat = bno.getQuat();
        quat = quat * rot_offset;
        imu::Vector<3> quat_euler = quat.toEuler();
        double hdg = RobotUtils::wrap_0_2pi(quat_euler.x());
        /* ---------- */
        double rot_diff = hdg - last_hdg;
        if (rot_diff < 0) rot_diff += 2 * PI;
        total_rot += rot_diff;

        if (odometry.update())
        {
            float wl, wr, dt, theta;
            odometry.getRobotPos(&wl, &wr, &theta, false)
            odometry.getWheelOmega(&wl, &wr, false);
            odometry.getTimeDelta(&dt);
            debug_msg.odom.pos.left = theta;
            debug_msg.odom.pos.right = total_rot;
            debug_msg.odom.omega.left = wl;
	        debug_msg.odom.omega.right = wr;
	        debug_msg.odom.time = dt;
	        debug_pub.publish(&debug_msg);
	        nh.spinOnce();
        }
    }
    motor_right.setSpeed(0);
	motor_left.setSpeed(0);
	nh.loginfo("Rotation calibration completed.");
}
