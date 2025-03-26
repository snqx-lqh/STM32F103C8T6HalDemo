#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include "main.h"

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float* IMU_Angle);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float* IMU_Angle);

#endif

