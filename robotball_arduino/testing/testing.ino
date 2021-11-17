#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
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
double g_pitch_sp = 0;
double g_speed_sp;
double g_hdg_sp;

double g_speed_odom;
double g_speed_extern;
// Select which source of speed measurements is used. DO NOT change it anywhere
// else in the code.
double *g_speed_used = &g_speed_odom;

double g_pitch;
double g_roll;
double g_hdg;
double g_hdg_offset;

double g_vel_lin;
double g_vel_rot;
double g_vel_input;

unsigned long last_cmd_vel;  // Last time velocity command was received

#define MODE_STABLE 0
#define MODE_DRIVE 1
int mode = MODE_STABLE;
bool PID_speed_enabled = false;
bool PID_pitch_enabled = false;

// v_ref -> pitch
PID PID_speed(g_speed_used, &g_vel_lin, &g_speed_sp, 0, 0, 0, 100, DIRECT);  // In, Out, Sp, Kp, Ki, Kd (2, 5, 1), Ts

// pitch -> motor_speed (linear.x)
PID PID_pitch(&g_pitch, &g_vel_lin, &g_pitch_sp, 0, 0, 0, 50, REVERSE); // In, Out, Sp, Kp, Ki, Kd (0.3/1.57, 0, 0), Ts

// hdg_ref -> motor_speed (angular.z)
PID PID_hdg(&g_hdg, &g_vel_rot, &g_hdg_sp, 0, 0, 0, 50, DIRECT);  // In, Out, Sp, Kp, Ki, Kd (2, 5, 1), Ts
/******************************************************************************/

/*************************** LOW-LEVEL STUFF **********************************/
const int8_t led_pin = 2;
const int8_t led_status = 1;

const int8_t bat_pin = A1;
/******************************************************************************/

/******************************* ROS SETUP ************************************/
// ros::NodeHandle_<ArduinoHardware, 5, 5, 1024, 1024> nh;
ros::NodeHandle nh;

robotball_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);

robotball_msgs::IMU imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

robotball_msgs::Status status_msg;
ros::Publisher status_pub("status", &status_msg);

robotball_msgs::Debug debug_msg;
ros::Publisher debug_pub("debug", &debug_msg);

void cmdVelCb (const geometry_msgs::Twist& cmd_vel) {
    if (PID_hdg.GetMode())
        // Heading controller is active. Joystick commands the desired heading.
        g_hdg_sp = cmd_vel.linear.y;
    else
        // Joystick commands the rotational part of motor mixer directly.
        g_vel_rot = cmd_vel.linear.y / (PI);

    if (PID_speed_enabled)
        // Speed controller is active. Joystick commands the desired speed.
        g_speed_sp = cmd_vel.linear.x * g_speed_scale;
    else
        // Joystick commands the linear part of motor mixer directly.
        g_vel_input = cmd_vel.linear.x * 1;

    if (cmd_vel.linear.z == 1)
        odometry.reset();

    last_cmd_vel = millis();
}
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &cmdVelCb);

void dynReconfCb (const robotball_msgs::DynReconf& msg) {
    PID_hdg.SetMode(msg.hdg.enabled);
    PID_pitch_enabled = msg.pitch.enabled;
    PID_speed_enabled = msg.speed.enabled;
    if (msg.pitch.enabled && mode == MODE_STABLE)
    {
        PID_pitch.SetMode(AUTOMATIC);
    }

    bool success = true;
    success &= PID_pitch.SetTunings(msg.pitch.P, msg.pitch.I, msg.pitch.D);
    success &= PID_speed.SetTunings(msg.speed.P, msg.speed.I, msg.speed.D);
    success &= PID_hdg.SetTunings(msg.hdg.P, msg.hdg.I, msg.hdg.D);
    g_hdg_offset = msg.hdg_offset * DEG_TO_RAD;

    if (success)
        nh.loginfo("PID parameters successfully changed.");
    else
        nh.logwarn("PID parameters couldn't be changed. They are negative or NaN.");
}
ros::Subscriber<robotball_msgs::DynReconf> reconf_sub("dyn_reconf", &dynReconfCb);

void externVelCb (const geometry_msgs::Vector3& msg) {
    float x = msg.x;
    float y = msg.y;

    g_speed_extern = sqrt(x*x + y*y);

}
ros::Subscriber<geometry_msgs::Vector3> vel_sub("vel_estimated", &externVelCb);


Timer<4, millis> timer;

void publish_imu() {
    imu_pub.publish(&imu_msg);
}

void publish_odom() {
    odom_pub.publish(&odom_msg);
}

void publish_status() {
    /* Get the robot status */
    // status_msg.battery = analogRead(bat_pin);
    status_pub.publish(&status_msg);
}

void publih_debug() {
    debug_pub.publish(&debug_msg);
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
    nh.advertise(imu_pub);
    nh.advertise(odom_pub);
    nh.advertise(debug_pub);
    nh.advertise(status_pub);
    nh.subscribe(cmd_sub);
    nh.subscribe(reconf_sub);
    nh.subscribe(vel_sub);
    nh.spinOnce();

    timer.every(1.0 / 1  * 1000, publish_status);
    timer.every(1.0 / 10 * 1000, publish_odom);
    timer.every(1.0 / 10 * 1000, publih_debug);
    timer.every(1.0 / 10 * 1000, publish_imu);

    /* Initialize BNO055 IMU. */
    while(!bno.begin())
    {
        // There was a problem detecting the BNO055.
        for(int i=0; i<5; i++) {
            digitalWrite(led_pin, HIGH);
            delay(500);
            digitalWrite(led_pin, LOW);
            delay(500);
            status_msg.calibration = -1;
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
    
    /* Set predefined PID parameters. */
    PID_pitch.SetOutputLimits(-1, 1);
    PID_speed.SetOutputLimits(-1, 1);
    PID_hdg.SetOutputLimits(-1, 1);

    /* Ready to start the main loop. */
    last_cmd_vel = millis();

}


void loop() {

    nh.spinOnce();

    /* Get the current IMU calibration status. */
    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    status_msg.calibration = 90000 + system * 1000 + gyro * 100 + accel * 10 + mag;
    /* ---------- */


    /* Get the latest orientation data. */
    imu::Quaternion quat = bno.getQuat();
    // IMU is mounted differently from its default orientation. In order to
    // avoid singularities at certain angles, we need to rotate the measured
    // quaternion before converting it to euler. We do that by multiplying it
    // with the magic quaternion below.
    quat = quat * rot_offset;
    // Axis on the IMU are wierdly flipped so we also need to flip x and z
    // axis for rotation. When all values are zero, the robot is in neutral
    // orientation and pointing towards east. */
    imu::Vector<3> quat_euler = quat.toEuler();
    g_roll = quat_euler.z();
    g_pitch = quat_euler.y();
    g_hdg = RobotUtils::wrap_pi_pi(quat_euler.x() - PI/2 + g_hdg_offset);

    imu_msg.orientation.x = quat.x();
    imu_msg.orientation.y = quat.y();
    imu_msg.orientation.z = quat.z();
    imu_msg.orientation.w = quat.w();
    imu_msg.euler.x = g_roll;
    imu_msg.euler.y = g_pitch;
    imu_msg.euler.z = g_hdg;
    /* ---------- */


    /* Handle yaw wrapping. */
    if (abs(g_hdg - g_hdg_sp) > PI)
    {
        if (g_hdg > 0) g_hdg_sp += 2 * PI;
        else g_hdg_sp -= 2 * PI;
    }

    /* Limit the speed setpoint in large turns to avoid excessive rolling. */
    if (PID_hdg.GetMode() == AUTOMATIC)
    {
        float hdg_err = fabs(g_hdg - g_hdg_sp);
        if (PID_speed_enabled)
        {
            float limit_speed_sp = fmap(hdg_err, 0, PI, g_speed_scale, 0);
            g_speed_sp = constrain(g_speed_sp, -limit_speed_sp, limit_speed_sp);
        }
        else
        {
            float limit_vel_lin = fmap(hdg_err, 0, PI, 1, 0);
            g_vel_lin = constrain(g_vel_lin, -limit_vel_lin, limit_vel_lin);
        }
    }
    /* ---------- */


    /* Get the latest odometry data. */
    double angular_odom;
    float pos_x, pos_y, pos_theta;
    odometry.update();
    odometry.getRobotVel(&g_speed_odom, &angular_odom);
    odometry.getRobotPos(&pos_x, &pos_y, &pos_theta);

    odom_msg.pose.x = pos_x;
    odom_msg.pose.y = pos_y;
    odom_msg.pose.theta = pos_theta;
    odom_msg.velocity.x = g_speed_odom;
    odom_msg.velocity.z = angular_odom;
    
    float left, right;
    odometry.getEncoderShift(&left, &right);
    debug_msg.odom.shift.left = left;
    debug_msg.odom.shift.right = right;
    odometry.getWheelPos(&left, &right);
    debug_msg.odom.pos.left = left;
    debug_msg.odom.pos.right = right;
    odometry.getWheelOmega(&left, &right);
    debug_msg.odom.omega.left = left;
    debug_msg.odom.omega.right = right;
    /* ---------- */


    /* Compute all PID outputs */
    // Select the current mode (pitch controller or speed controller active).
    if (mode == MODE_STABLE)
    {
        if ((PID_speed_enabled && g_speed_sp != 0) || (!PID_speed_enabled && g_vel_input != 0))
        {
            mode = MODE_DRIVE;
            PID_speed.SetMode(PID_speed_enabled ? AUTOMATIC : MANUAL);
            PID_pitch.SetMode(MANUAL);
        }
    }
    else  // mode == MODE_DRIVE
    {
        g_vel_lin = g_vel_input;
        if ((PID_speed_enabled && g_speed_sp == 0) || (!PID_speed_enabled && g_vel_input == 0))
        {
            if (g_speed_odom < 0.05)
            {
                mode = MODE_STABLE;
                PID_speed.SetMode(MANUAL);
                PID_pitch.SetMode(PID_pitch_enabled ? AUTOMATIC : MANUAL);
            }
        }
    }


    // Compute the outputs. Inactive controllers are automatically skipped.
    PID_hdg.Compute();
    PID_speed.Compute();
    PID_pitch.Compute();

    debug_msg.speed.setpoint = g_speed_sp;
    debug_msg.speed.measured = *g_speed_used;
    debug_msg.speed.output   = g_vel_lin;
    debug_msg.pitch.setpoint = g_pitch_sp;
    debug_msg.pitch.measured = g_pitch;
    debug_msg.pitch.output   = g_vel_lin;
    debug_msg.hdg.setpoint   = g_hdg_sp;
    debug_msg.hdg.measured   = g_hdg;
    debug_msg.hdg.output     = g_vel_rot;
    debug_msg.mode           = mode;
    /* ---------- */


    /* Control the motors. */
    double vel_left = 0;
    double vel_right = 0;

    vel_left = g_vel_lin - g_vel_rot;
    vel_right = g_vel_lin + g_vel_rot;

    debug_msg.vel.left = g_vel_lin;
    debug_msg.vel.right = g_vel_lin;

    double scale_factor = 1.0;
    double abs_left = fabs(vel_left);
    double abs_right = fabs(vel_right);
    if ((abs_left > 1) || (abs_right > 1))
    {
        double x = max(abs_left, abs_right);
        scale_factor = 1.0 / x;
    }

    vel_left = round(vel_left * scale_factor * 255);
    vel_right = round(vel_right * scale_factor * 255);

    if (millis() - last_cmd_vel < cmd_vel_timeout)
    {
        motor_right.setSpeed(r_motor_sign * vel_right);
        motor_left.setSpeed(l_motor_sign * vel_left);  
    }
    else
    {
        motor_right.setSpeed(0);
        motor_left.setSpeed(0);  
    }

    debug_msg.motor.left  = vel_left;
    debug_msg.motor.right = vel_right;
    /* ---------- */

    // Publish all ROS messages according to their rates.
    timer.tick();
}
