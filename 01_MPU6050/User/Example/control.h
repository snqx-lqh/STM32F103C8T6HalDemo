#ifndef _CONTROL_H
#define _CONTROL_H

#include "main.h"

typedef struct{
	float accxAngle;
	float accyAngle;
	float acczAngle;
	
	float gyroxReal;
	float gyroyReal;
	float gyrozReal;
	
	float accxReal;
	float accyReal;
	float acczReal;
	
	float angle[3];
	
	float angleRoll;
	float anglePitch;
	float angleYaw;
	
	int16_t gyro[3];
	int16_t acc[3];
}mpu6050_data_t;

void exit_update(void);
void app_run_main(void);

#endif
