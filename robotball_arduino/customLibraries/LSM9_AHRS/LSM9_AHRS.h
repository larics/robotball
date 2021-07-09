#ifndef _LSM9_AHRS_H_
#define _LSM9_AHRS_H_

bool AHRS_setup();
void AHRS_update(double*, double*, double*);

#endif
