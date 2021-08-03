#include "RobotUtilities.h"

namespace RobotUtils
{
	/* wrap x -> [0,2*PI) */
	double wrap_0_2pi(double x)
	{
	    // return fmod(max + fmod(x, max), max);
	    return fmod(2*M_PI + fmod(x, 2*M_PI), 2*M_PI);
	}
	
	/* wrap x -> [-PI,PI) */
	double wrap_pi_pi(double x)
	{
	    //return min + wrap_0_2pi(x - min, max - min);
	    return -M_PI + wrap_0_2pi(x + M_PI);
	}

	double prepare_yaw(double setpoint, double measured)
	{
		double new_setpoint = wrap_pi_pi(setpoint);

		if (fabs(new_setpoint - measured) > M_PI)
		{
			if (measured > 0)
				new_setpoint += 2*M_PI;
			else
				new_setpoint -= 2*M_PI;	
		}

		return new_setpoint;
	}

	void multiply_quaternions(float *w1, float *x1, float *y1, float *z1,
							  float w2, float x2, float y2, float z2)
	{
		float w = *w1 * w2 - *x1 * x2 - *y1 * y2 - *z1 * z2;
	    float x = *w1 * x2 + *x1 * w2 + *y1 * z2 - *z1 * y2;
	    float y = *w1 * y2 - *x1 * z2 + *y1 * w2 + *z1 * x2;
	    float z = *w1 * z2 + *x1 * y2 - *y1 * x2 + *z1 * w2;

	    *w1 = w;
	    *x1 = x;
	    *y1 = y;
	    *z1 = z;
	}
}