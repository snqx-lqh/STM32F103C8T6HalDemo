#ifndef _INV_MPU_STM32PORT_H
#define _INV_MPU_STM32PORT_H

#include "main.h"

int mpu_dmp_init(void);
int mpu_dmp_get_data(float *pitch, float *roll, float *yaw);

#endif
