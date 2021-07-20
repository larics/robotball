#ifndef _ROBOT_UTILS_H_
#define _ROBOT_UTILS_H_

#include <math.h>

namespace RobotUtils
{
	/* wrap x -> [0,2*PI) */
	double wrap_0_2pi(double);

	/* wrap x -> [-PI,PI) */
	double wrap_pi_pi(double);
}


#endif