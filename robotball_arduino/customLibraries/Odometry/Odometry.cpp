#include "Odometry.h"


DiffDriveOdom::DiffDriveOdom(EncoderPins pins, int encoder_rate, DiffDriveParams params)
{
    // Initialize the encoders.
    enc_left_= new Encoder(pins.left_1, pins.left_2);
    enc_right_ = new Encoder(pins.right_1, pins.right_2);

    // Save the parameters.
    COUNTER_PER_ROTATION_ = encoder_rate;
    lR_ = fabs(params.lwr);
    rR_ = fabs(params.rwr);
    b_ = params.ws;
    k1_ = params.k1;
    k2_ = params.k2;
    k3_ = params.k3;
    transmission_ = params.sr / params.cr;
    lDir_ = params.lwr < 0 ? -1 : 1;
    rDir_ = params.rwr < 0 ? -1 : 1;

    // Initialize other variables.
    // Last update time and encoder positions
    last_update_ = millis();
    time_delta_ = 0;

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

bool DiffDriveOdom::update()
{
    // Calculate the time elapsed since last update.
    unsigned long now = millis();
    time_delta_ = (now - last_update_) / 1000.0;
    if (time_delta_ < 0.100)
        return false;
    last_update_ = now;

    // Read the encoder offset since last update.
    int left_pos_delta = lDir_ * enc_left_->readAndReset();
    int right_pos_delta = rDir_ * enc_right_->readAndReset();

    // Calculate the position of encoders.
    pos_l_ += (left_pos_delta / COUNTER_PER_ROTATION_ * TWO_PI);
    pos_r_ += (right_pos_delta / COUNTER_PER_ROTATION_ * TWO_PI);

    // Calculate angular speed of both wheels.
    omega_l_ = (left_pos_delta / COUNTER_PER_ROTATION_) * TWO_PI / time_delta_;
    omega_r_ = (right_pos_delta / COUNTER_PER_ROTATION_) * TWO_PI / time_delta_;
        
    // Calculate angular and linear velocity of the robot.
    angular_ = (-omega_l_ * lR_ * k1_ + omega_r_ * rR_ * k2_) / (b_ * k3_);
    linear_ = ( (omega_l_ * lR_ * k1_ + omega_r_ * rR_ * k2_) / 2 ) * transmission_;
    
    // Calculate the position of the robot in (x,y) coordinates.
    theta_ = theta_ + angular_ * time_delta_;
    x_ = x_ + linear_ * time_delta_ * cos(theta_);
    y_ = y_ + linear_ * time_delta_ * sin(theta_);

    return true;
}


void DiffDriveOdom::getEncoderShift(float* left_shift, float* right_shift, bool degrees)
{
    *left_shift = pos_l_ * (degrees ? RAD_TO_DEG : 1);
    *right_shift = pos_r_ * (degrees ? RAD_TO_DEG : 1);
}

void DiffDriveOdom::getWheelPos(float* left_pos, float* right_pos, bool degrees)
{
    *left_pos = RobotUtils::wrap_0_2pi(pos_l_) * (degrees ? RAD_TO_DEG : 1);
    *right_pos = RobotUtils::wrap_0_2pi(pos_r_) * (degrees ? RAD_TO_DEG : 1);
}

void DiffDriveOdom::getWheelOmega(float* left_omega, float* right_omega, bool degrees)
{
    *left_omega = omega_l_ * (degrees ? RAD_TO_DEG : 1);
    *right_omega = omega_r_ * (degrees ? RAD_TO_DEG : 1);
}

void DiffDriveOdom::getRobotVel(double* linear, double* angular, bool degrees)
{
    *linear = (double) linear_;
    *angular = (double) (angular_ * (degrees ? RAD_TO_DEG : 1));
}

void DiffDriveOdom::getRobotPos(float* x, float* y, float* theta, bool degrees)
{
    *x = x_;
    *y = y_;
    *theta = theta_ * (degrees ? RAD_TO_DEG : 1);
}

void DiffDriveOdom::getTimeDelta(float* dt)
{
    *dt = time_delta_;
}

void DiffDriveOdom::reset()
{
    // Last update time and encoder positions
    last_update_ = millis();

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

    // Also reset the encoders.
    enc_left_->write(0);
    enc_right_->write(0);
}
