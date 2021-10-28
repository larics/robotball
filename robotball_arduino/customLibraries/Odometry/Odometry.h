#ifndef dif_robot_odometry_h_
#define dif_robot_odometry_h_

#include <Arduino.h>
#include <Encoder.h>
#include <math.h>
#include "RobotUtilities.h"

typedef struct {
    byte left_1;
    byte left_2;
    byte right_1;
    byte right_2;
} EncoderPins;

typedef struct {
    float lwr;  // Left wheel radius (m)
    float rwr;  // Right wheel radius (m)
    float ws;   // Wheel separation (m)
    float sr;   // Outer shell radius (m)
    float cr;   // Radius of the contact point between wheels and inner shell (m)
} DiffDriveParams;


class DiffDriveOdom
{
    public: 
        DiffDriveOdom(EncoderPins pins, int encoder_rate, DiffDriveParams params);
        void reset();
        void update();
        void getEncoderShift(float* left_shift, float* right_shift, bool degrees = false);
        void getWheelPos(float* left_pos, float* right_pos, bool degrees = false);
        void getWheelOmega(float* left_omega, float* right_omega, bool degrees = false);
        void getRobotVel(double* linear, double* angular, bool degrees = false);
        void getRobotPos(float* x, float* y, float* theta, bool degrees = false);

    private:
        // Encoder hardware
        Encoder* enc_left_;
        Encoder* enc_right_;
        float COUNTER_PER_ROTATION_;

        // Last update time
        unsigned long last_update_;

        // Velocities and positions of the wheels
        float pos_l_;
        float pos_r_;
        float omega_l_;
        float omega_r_;

        // Velocities and pose of the robot
        float x_;
        float y_;
        float theta_;
        float angular_;
        float linear_;
        
        // Robot parameters
        float lR_;
        float rR_;
        float b_;
        float transmission_;
        int8_t lDir_;
        int8_t rDir_;

};

#endif