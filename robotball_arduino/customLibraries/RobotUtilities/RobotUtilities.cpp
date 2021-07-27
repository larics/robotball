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
}