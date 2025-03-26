#ifndef MahonyAHRS_h
#define MahonyAHRS_h

#include "main.h"

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float* IMU_Angle);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float* IMU_Angle);

#endif

