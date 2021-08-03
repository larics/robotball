#ifndef _ROBOT_UTILS_H_
#define _ROBOT_UTILS_H_

#include <math.h>

namespace RobotUtils
{
	/* wrap x -> [0,2*PI) */
	double wrap_0_2pi(double);

	/* wrap x -> [-PI,PI) */
	double wrap_pi_pi(double);

	double prepare_yaw(double, double);

	void multiply_quaternions(float *w1, float *x1, float *y1, float *z1,
							  float w2, float x2, float y2, float z2);
}


#endif