#ifndef dif_robot_odometry_h_
#define dif_robot_odometry_h_

#include <Arduino.h>
#include <Encoder.h>
#include <math.h>

typedef struct {
    int left_1;
    int left_2;
    int right_1;
    int right_2;
} EncoderPins;

typedef struct {
    float lwr;  // Left wheel radius (m)
    float rwr;  // Right wheel radius (m)
    float ws;   // Wheel separation (m)
} DiffDriveParams;


class DiffDriveOdom
{
    public: 
        DiffDriveOdom(EncoderPins pins, int encoder_rate, DiffDriveParams params);
        void reset();
        void update();
        void getEncoderShift(float* left_shift, float* right_shift, bool degrees);
        void getWheelPos(float* left_pos, float* right_pos, bool degrees);
        void getWheelOmega(float* left_omega, float* right_omega, bool degrees);
        void getRobotVel(float* linear, float* angular, bool degrees);
        void getRobotPos(float* x, float* y, float* theta, bool degrees);
        static double wrap_0_2pi(double x);
        static double wrap_pi_pi(double x);

    private:
        // Encoder hardware
        Encoder* enc_left_;
        Encoder* enc_right_;
        float COUNTER_PER_ROTATION_;

        // Last update time and encoder positions
        unsigned long last_update_;
        long encoder_ticks_left_;
        long encoder_ticks_right_;

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
};

#endif