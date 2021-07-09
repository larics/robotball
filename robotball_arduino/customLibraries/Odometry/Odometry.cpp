#include "Odometry.h"


DiffDriveOdom::DiffDriveOdom(EncoderPins pins, int encoder_rate, DiffDriveParams params)
{
    // Initialize the encoders.
    enc_left_= new Encoder(pins.left_1, pins.left_2);
    enc_right_ = new Encoder(pins.right_1, pins.right_2);

    // Save the parameters.
    COUNTER_PER_ROTATION_ = encoder_rate;
    lR_ = params.lwr;
    rR_ = params.rwr;
    b_ = params.ws;

    // Initialize other variables.
    // Last update time and encoder positions
    last_update_ = millis();
    encoder_ticks_left_ = 0;
    encoder_ticks_right_ = 0;

    // Velocities and positions of the wheels
    pos_l_ = 0;
    pos_r_ = 0;
    omega_l_ = 0;
    omega_r_ = 0;

    // Velocities and pose of the robot
    x_ = 0;
    y_ = 0;
    theta_ = 0;
    angular_ = 0;
    linear_ = 0;
}

void DiffDriveOdom::update()
{
    // Read current encoder positions.
    long new_ticks_left = enc_left_->read();
    long new_ticks_right = enc_right_->read();

    // Calculate the time elapsed since last update.
    unsigned long now = millis();
    float time_delta = (now - last_update_) / 1000.0;

    if (time_delta < 0.100)
        return;
    last_update_ = now;

    // Calculate the encoder offset since last update.
    long left_pos_delta = new_ticks_left - encoder_ticks_left_;
    long right_pos_delta = new_ticks_right - encoder_ticks_right_;
    encoder_ticks_left_ = new_ticks_left;
    encoder_ticks_right_ = new_ticks_right;

    // Calculate the position of encoders.
    pos_l_ = new_ticks_left / COUNTER_PER_ROTATION_ * TWO_PI;
    pos_r_ = new_ticks_right / COUNTER_PER_ROTATION_ * TWO_PI;

    // Calculate angular speed of both wheels.
    omega_l_ = (left_pos_delta / COUNTER_PER_ROTATION_) * TWO_PI / time_delta;
    omega_r_ = (right_pos_delta / COUNTER_PER_ROTATION_) * TWO_PI / time_delta;
        
    // Calculate angular and linear velocity of the robot.
    angular_ = (-omega_l_ * lR_ + omega_r_ * rR_) / b_;
    linear_ = (omega_l_ * lR_ + omega_r_ * rR_) / 2; 
    
    // Calculate the position of the robot in (x,y) coordinates.
    theta_ = theta_ + angular_ * time_delta;
    x_ = x_ + linear_ * time_delta * cos(theta_);
    y_ = y_ + linear_ * time_delta * sin(theta_);
}


void DiffDriveOdom::getEncoderShift(float* left_shift, float* right_shift, bool degrees)
{
    *left_shift = pos_l_ * (degrees ? RAD_TO_DEG : 1);
    *right_shift = pos_r_ * (degrees ? RAD_TO_DEG : 1);
}

void DiffDriveOdom::getWheelPos(float* left_pos, float* right_pos, bool degrees)
{
    *left_pos = wrap_0_2pi(pos_l_) * (degrees ? RAD_TO_DEG : 1);
    *right_pos = wrap_0_2pi(pos_r_) * (degrees ? RAD_TO_DEG : 1);
}

void DiffDriveOdom::getWheelOmega(float* left_omega, float* right_omega, bool degrees)
{
    *left_omega = omega_l_ * (degrees ? RAD_TO_DEG : 1);
    *right_omega = omega_r_ * (degrees ? RAD_TO_DEG : 1);
}

void DiffDriveOdom::getRobotVel(float* linear, float* angular, bool degrees)
{
    *linear = linear_;
    *angular = angular_ * (degrees ? RAD_TO_DEG : 1);
}

void DiffDriveOdom::getRobotPos(float* x, float* y, float* theta, bool degrees)
{
    *x = x_;
    *y = y_;
    *theta = theta_ * (degrees ? RAD_TO_DEG : 1);
}

void DiffDriveOdom::reset()
{
    // Last update time and encoder positions
    last_update_ = millis();
    encoder_ticks_left_ = 0;
    encoder_ticks_right_ = 0;

    // Velocities and positions of the wheels
    pos_l_ = 0;
    pos_r_ = 0;
    omega_l_ = 0;
    omega_r_ = 0;

    // Velocities and pose of the robot
    x_ = 0;
    y_ = 0;
    theta_ = 0;
    angular_ = 0;
    linear_ = 0;
}

/* wrap x -> [0,2*PI) */
double DiffDriveOdom::wrap_0_2pi(double x)
{
    // return fmod(max + fmod(x, max), max);
    return fmod(2*M_PI + fmod(x, 2*M_PI), 2*M_PI);
}
/* wrap x -> [-PI,PI) */
double DiffDriveOdom::wrap_pi_pi(double x)
{
    //return min + wrap_0_2pi(x - min, max - min);
    return -M_PI + DiffDriveOdom::wrap_0_2pi(x + M_PI);
}