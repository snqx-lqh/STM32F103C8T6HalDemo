#include "control.h"
#include "stdio.h"

#include "mpu9250.h"
#include "KalmanFilter.h"
#include "FirstOrderLowPassFilter.h"
#include "MahonyAHRS.h"
#include "MadgwickAHRS.h"
#include "math.h"
 
#include "inv_mpu.h"
#include "inv_mpu_stm32port.h"

#define PI 3.1415926

static volatile uint8_t data_ready = 0;

mpu9250_data_t mpu9250_data;

static void cal_with_kalman(void);
static void cal_with_folpf(void);
static void cal_with_ahrs(void);
static void cal_with_dmp(void);

/**
  * @brief   中断更新信号，该部分处理放置在中断引脚的中断处理中，本代码放在main.c中的
  *          不知道为啥，9250的中断引脚一直有问题，所以并未使用中断引脚的相关处理，而是直接延时读取
  *          但是更好的方案应该是定时器处理，但是我没有做相关的处理，可以后续自行添加。
  * @param   
  * @retval  void
 **/
void exit_update()
{
	data_ready = 1;
}

/**
  * @brief   执行控制任务，该部分代码放置在main.c的while前
  * @param    
  * @retval  void
 **/
void app_run_main()
{
//	cal_with_kalman();
//	cal_with_folpf();
	cal_with_ahrs();
//	cal_with_dmp();
}
/**
  * @brief   使用kalman滤波计算角度
  * @param    
  * @retval  void
 **/
static void cal_with_kalman()
{
	int count = 0;
	mpu9250_init();
	while(1)
	{
		HAL_Delay(10);
		//获得9250原始数据
		mpu9250_get_gyro(&mpu9250_data.gyro[0],&mpu9250_data.gyro[1],&mpu9250_data.gyro[2]);
		mpu9250_get_acc(&mpu9250_data.acc[0],&mpu9250_data.acc[1],&mpu9250_data.acc[2]);
		
		//做单位解算 量程转换
		mpu9250_data.gyroxReal = mpu9250_data.gyro[0] * MPU9250_GYRO_2000_SEN;
		mpu9250_data.gyroyReal = mpu9250_data.gyro[1] * MPU9250_GYRO_2000_SEN;
		mpu9250_data.gyrozReal = mpu9250_data.gyro[2] * MPU9250_GYRO_2000_SEN;
		mpu9250_data.accxReal  = mpu9250_data.acc[0]  * MPU9250_ACCEL_2G_SEN;
		mpu9250_data.accyReal  = mpu9250_data.acc[1]  * MPU9250_ACCEL_2G_SEN;
		mpu9250_data.acczReal  = mpu9250_data.acc[2]  * MPU9250_ACCEL_2G_SEN;
		
		//计算角度 卡尔曼滤波方式
		mpu9250_data.accyAngle=atan2(mpu9250_data.acc[0],mpu9250_data.acc[2])*180/PI;  //加速度计算倾角	
		Kalman_getAngle(&KalmanY,mpu9250_data.accyAngle,-mpu9250_data.gyroyReal,0.01); //卡尔曼滤波算角度
		mpu9250_data.anglePitch = KalmanY.angle;
		
		mpu9250_data.accxAngle=atan2(mpu9250_data.acc[1],mpu9250_data.acc[2])*180/PI;  //加速度计算倾角	
		Kalman_getAngle(&KalmanX,mpu9250_data.accxAngle,-mpu9250_data.gyroxReal,0.01); //卡尔曼滤波算角度
		mpu9250_data.angleRoll = KalmanX.angle;
		
		count ++;
		if(count % 100 == 0)
		{
			printf("%f, %f, %f\r\n",mpu9250_data.anglePitch,mpu9250_data.angleRoll,mpu9250_data.angleYaw);
		}
	}
}

/**
  * @brief   使用互补滤波计算角度
  * @param    
  * @retval  void
 **/
static void cal_with_folpf()
{
	int count = 0;
	mpu9250_init();
	while(1)
	{
		HAL_Delay(10);
		//获得9250原始数据
		mpu9250_get_gyro(&mpu9250_data.gyro[0],&mpu9250_data.gyro[1],&mpu9250_data.gyro[2]);
		mpu9250_get_acc(&mpu9250_data.acc[0],&mpu9250_data.acc[1],&mpu9250_data.acc[2]);
		
		//做单位解算 量程转换
		mpu9250_data.gyroxReal = mpu9250_data.gyro[0] * MPU9250_GYRO_2000_SEN;
		mpu9250_data.gyroyReal = mpu9250_data.gyro[1] * MPU9250_GYRO_2000_SEN;
		mpu9250_data.gyrozReal = mpu9250_data.gyro[2] * MPU9250_GYRO_2000_SEN;
		mpu9250_data.accxReal  = mpu9250_data.acc[0]  * MPU9250_ACCEL_2G_SEN;
		mpu9250_data.accyReal  = mpu9250_data.acc[1]  * MPU9250_ACCEL_2G_SEN;
		mpu9250_data.acczReal  = mpu9250_data.acc[2]  * MPU9250_ACCEL_2G_SEN;
		
		//计算角度 互补滤波方式
		mpu9250_data.accyAngle=atan2(mpu9250_data.acc[0],mpu9250_data.acc[2])*180/PI;  //加速度计算倾角	
		FirstOrderLowPassFilter(&FOLPF_angley,mpu9250_data.accyAngle,-mpu9250_data.gyroyReal,0.01); //互补滤波算角度
		mpu9250_data.anglePitch = FOLPF_angley.angle;
		
		mpu9250_data.accxAngle=atan2(mpu9250_data.acc[1],mpu9250_data.acc[2])*180/PI;  //加速度计算倾角	
		FirstOrderLowPassFilter(&FOLPF_anglex,mpu9250_data.accxAngle,-mpu9250_data.gyroxReal,0.01); //互补滤波算角度
		mpu9250_data.angleRoll = FOLPF_anglex.angle;
		
		count ++;
		if(count % 100 == 0)
		{
			printf("%f, %f, %f\r\n",mpu9250_data.anglePitch,mpu9250_data.angleRoll,mpu9250_data.angleYaw);
		}
	}
}

/**
  * @brief   使用AHRS算法计算角度
  * @param    
  * @retval  void
 **/
static void cal_with_ahrs()
{
	int count = 0;
	mpu9250_init();
	while(1)
	{
		HAL_Delay(10);
		//获得9250原始数据
		mpu9250_get_gyro(&mpu9250_data.gyro[0],&mpu9250_data.gyro[1],&mpu9250_data.gyro[2]);
		mpu9250_get_acc(&mpu9250_data.acc[0],&mpu9250_data.acc[1],&mpu9250_data.acc[2]);
		mpu9250_get_mag(&mpu9250_data.mag[0],&mpu9250_data.mag[1],&mpu9250_data.mag[2]);
		
		//做单位解算 量程转换
		mpu9250_data.gyroxReal = mpu9250_data.gyro[0] * MPU9250_GYRO_2000_SEN;
		mpu9250_data.gyroyReal = mpu9250_data.gyro[1] * MPU9250_GYRO_2000_SEN;
		mpu9250_data.gyrozReal = mpu9250_data.gyro[2] * MPU9250_GYRO_2000_SEN;
		mpu9250_data.accxReal  = mpu9250_data.acc[0]  * MPU9250_ACCEL_2G_SEN;
		mpu9250_data.accyReal  = mpu9250_data.acc[1]  * MPU9250_ACCEL_2G_SEN;
		mpu9250_data.acczReal  = mpu9250_data.acc[2]  * MPU9250_ACCEL_2G_SEN;
		mpu9250_data.magxReal  = mpu9250_data.mag[0]  * MPU9250_MAG_SEN;
		mpu9250_data.magyReal  = mpu9250_data.mag[1]  * MPU9250_MAG_SEN;
		mpu9250_data.magzReal  = mpu9250_data.mag[2]  * MPU9250_MAG_SEN;
		
		//计算角度值 Mahony解算
		MahonyAHRSupdate(mpu9250_data.gyroxReal,mpu9250_data.gyroyReal,mpu9250_data.gyrozReal, \
						 mpu9250_data.accxReal,mpu9250_data.accyReal,mpu9250_data.acczReal,    \
						 mpu9250_data.magxReal, mpu9250_data.magyReal, mpu9250_data.magzReal, mpu9250_data.angle);
		//计算角度值 Madgwick解算
//			MadgwickAHRSupdate(mpu9250_data.gyroxReal,mpu9250_data.gyroyReal,mpu9250_data.gyrozReal, \
//							   mpu9250_data.accxReal,mpu9250_data.accyReal,mpu9250_data.acczReal,    \
//							   0, 0, 0, mpu9250_data.angle);
		
		mpu9250_data.angleRoll  = mpu9250_data.angle[0];
		mpu9250_data.anglePitch = mpu9250_data.angle[1];
		mpu9250_data.angleYaw   = mpu9250_data.angle[2];
		
		count ++;
		if(count % 100 == 0)
		{
			printf("%f, %f, %f\r\n",mpu9250_data.anglePitch,mpu9250_data.angleRoll,mpu9250_data.angleYaw);
		}
	}
}
/**
  * @brief   使用DMP库计算角度
  * @param    
  * @retval  void
 **/
static void cal_with_dmp()
{
	int count = 0;
	unsigned long timestamp;
	mpu_dmp_init();
	while(1)
	{
		HAL_Delay(10);
		if(mpu_dmp_get_data(&mpu9250_data.anglePitch,&mpu9250_data.angleRoll,&mpu9250_data.angleYaw)==0)
		{ 
			mpu9250_get_gyro(&mpu9250_data.gyro[0],&mpu9250_data.gyro[1],&mpu9250_data.gyro[2]);
			mpu9250_get_acc (&mpu9250_data.acc[0] ,&mpu9250_data.acc[1] ,&mpu9250_data.acc[2]);
			//mpu9250_get_mag(&mpu9250_data.mag[0],&mpu9250_data.mag[1],&mpu9250_data.mag[2]);
			mpu_get_compass_reg(mpu9250_data.mag, &timestamp);
			//做单位解算 量程转换
			mpu9250_data.gyroxReal = mpu9250_data.gyro[0] * MPU9250_GYRO_2000_SEN;
			mpu9250_data.gyroyReal = mpu9250_data.gyro[1] * MPU9250_GYRO_2000_SEN;
			mpu9250_data.gyrozReal = mpu9250_data.gyro[2] * MPU9250_GYRO_2000_SEN;
			mpu9250_data.accxReal  = mpu9250_data.acc[0]  * MPU9250_ACCEL_2G_SEN;
			mpu9250_data.accyReal  = mpu9250_data.acc[1]  * MPU9250_ACCEL_2G_SEN;
			mpu9250_data.acczReal  = mpu9250_data.acc[2]  * MPU9250_ACCEL_2G_SEN;

			count ++;
			if(count % 100 == 0)
			{
				printf("%f, %f, %f\r\n",mpu9250_data.anglePitch,mpu9250_data.angleRoll,mpu9250_data.angleYaw);
			}
		}
	}
}



