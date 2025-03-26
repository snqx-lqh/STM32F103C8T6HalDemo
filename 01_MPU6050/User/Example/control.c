#include "control.h"
#include "stdio.h"

#include "mpu6050.h"
#include "KalmanFilter.h"
#include "FirstOrderLowPassFilter.h"
#include "MahonyAHRS.h"
#include "MadgwickAHRS.h"
#include "math.h"
#include "inv_mpu_stm32port.h"

#define PI 3.1415926

static volatile uint8_t data_ready = 0;

mpu6050_data_t mpu6050_data;

static void cal_with_kalman(void);
static void cal_with_folpf(void);
static void cal_with_ahrs(void);
static void cal_with_dmp(void);

/**
  * @brief   �жϸ����źţ��ò��ִ���������ж����ŵ��жϴ����У����������main.c�е�
  * @param   
  * @retval  void
 **/
void exit_update()
{
	data_ready = 1;
}

/**
  * @brief   ִ�п������񣬸ò��ִ��������main.c��whileǰ
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
  * @brief   ʹ��kalman�˲�����Ƕ�
  * @param    
  * @retval  void
 **/
static void cal_with_kalman()
{
	int count = 0;
	mpu6050_init();
	while(1)
	{
		//�ò��ָ��»�ͳ�ʼ��mpu6050ʱ��Ķ���Ĳ��������
		if(1 == data_ready)
		{
			data_ready = 0;
			//���6050ԭʼ����
			mpu6050_get_gyro(&mpu6050_data.gyro[0],&mpu6050_data.gyro[1],&mpu6050_data.gyro[2]);
			mpu6050_get_acc(&mpu6050_data.acc[0],&mpu6050_data.acc[1],&mpu6050_data.acc[2]);
			
			//����λ���� ����ת��
			mpu6050_data.gyroxReal = mpu6050_data.gyro[0] * MPU6050_GYRO_2000_SEN;
			mpu6050_data.gyroyReal = mpu6050_data.gyro[1] * MPU6050_GYRO_2000_SEN;
			mpu6050_data.gyrozReal = mpu6050_data.gyro[2] * MPU6050_GYRO_2000_SEN;
			mpu6050_data.accxReal  = mpu6050_data.acc[0]  * MPU6050_ACCEL_2G_SEN;
			mpu6050_data.accyReal  = mpu6050_data.acc[1]  * MPU6050_ACCEL_2G_SEN;
			mpu6050_data.acczReal  = mpu6050_data.acc[2]  * MPU6050_ACCEL_2G_SEN;
			
			//����Ƕ� �������˲���ʽ
			mpu6050_data.accyAngle=atan2(mpu6050_data.acc[0],mpu6050_data.acc[2])*180/PI;  //���ٶȼ������	
			Kalman_getAngle(&KalmanY,mpu6050_data.accyAngle,-mpu6050_data.gyroyReal,0.01); //�������˲���Ƕ�
			mpu6050_data.anglePitch = KalmanY.angle;
			
			mpu6050_data.accxAngle=atan2(mpu6050_data.acc[1],mpu6050_data.acc[2])*180/PI;  //���ٶȼ������	
			Kalman_getAngle(&KalmanX,mpu6050_data.accxAngle,-mpu6050_data.gyroxReal,0.01); //�������˲���Ƕ�
			mpu6050_data.angleRoll = KalmanX.angle;
			
			count ++;
			if(count % 100 == 0)
			{
				printf("%f, %f, %f\r\n",mpu6050_data.anglePitch,mpu6050_data.angleRoll,mpu6050_data.angleYaw);
			}
		}
	}
}

/**
  * @brief   ʹ�û����˲�����Ƕ�
  * @param    
  * @retval  void
 **/
static void cal_with_folpf()
{
	int count = 0;
	mpu6050_init();
	while(1)
	{
		//�ò��ָ��»�ͳ�ʼ��mpu6050ʱ��Ķ���Ĳ��������
		if(1 == data_ready)
		{
			data_ready = 0;
			//���6050ԭʼ����
			mpu6050_get_gyro(&mpu6050_data.gyro[0],&mpu6050_data.gyro[1],&mpu6050_data.gyro[2]);
			mpu6050_get_acc(&mpu6050_data.acc[0],&mpu6050_data.acc[1],&mpu6050_data.acc[2]);
			
			//����λ���� ����ת��
			mpu6050_data.gyroxReal = mpu6050_data.gyro[0] * MPU6050_GYRO_2000_SEN;
			mpu6050_data.gyroyReal = mpu6050_data.gyro[1] * MPU6050_GYRO_2000_SEN;
			mpu6050_data.gyrozReal = mpu6050_data.gyro[2] * MPU6050_GYRO_2000_SEN;
			mpu6050_data.accxReal  = mpu6050_data.acc[0]  * MPU6050_ACCEL_2G_SEN;
			mpu6050_data.accyReal  = mpu6050_data.acc[1]  * MPU6050_ACCEL_2G_SEN;
			mpu6050_data.acczReal  = mpu6050_data.acc[2]  * MPU6050_ACCEL_2G_SEN;
			
			//����Ƕ� �����˲���ʽ
			mpu6050_data.accyAngle=atan2(mpu6050_data.acc[0],mpu6050_data.acc[2])*180/PI;  //���ٶȼ������	
			FirstOrderLowPassFilter(&FOLPF_angley,mpu6050_data.accyAngle,-mpu6050_data.gyroyReal,0.01); //�����˲���Ƕ�
			mpu6050_data.anglePitch = FOLPF_angley.angle;
			
			mpu6050_data.accxAngle=atan2(mpu6050_data.acc[1],mpu6050_data.acc[2])*180/PI;  //���ٶȼ������	
			FirstOrderLowPassFilter(&FOLPF_anglex,mpu6050_data.accxAngle,-mpu6050_data.gyroxReal,0.01); //�����˲���Ƕ�
			mpu6050_data.angleRoll = FOLPF_anglex.angle;
			
			count ++;
			if(count % 100 == 0)
			{
				printf("%f, %f, %f\r\n",mpu6050_data.anglePitch,mpu6050_data.angleRoll,mpu6050_data.angleYaw);
			}
		}
	}
}

/**
  * @brief   ʹ��AHRS�㷨����Ƕ�
  * @param    
  * @retval  void
 **/
static void cal_with_ahrs()
{
	int count = 0;
	mpu6050_init();
	while(1)
	{
		//�ò��ָ��»�ͳ�ʼ��mpu6050ʱ��Ķ���Ĳ��������
		if(1 == data_ready)
		{
			data_ready = 0;
			//���6050ԭʼ����
			mpu6050_get_gyro(&mpu6050_data.gyro[0],&mpu6050_data.gyro[1],&mpu6050_data.gyro[2]);
			mpu6050_get_acc(&mpu6050_data.acc[0],&mpu6050_data.acc[1],&mpu6050_data.acc[2]);
			
			//����λ���� ����ת��
			mpu6050_data.gyroxReal = mpu6050_data.gyro[0] * MPU6050_GYRO_2000_SEN;
			mpu6050_data.gyroyReal = mpu6050_data.gyro[1] * MPU6050_GYRO_2000_SEN;
			mpu6050_data.gyrozReal = mpu6050_data.gyro[2] * MPU6050_GYRO_2000_SEN;
			mpu6050_data.accxReal  = mpu6050_data.acc[0]  * MPU6050_ACCEL_2G_SEN;
			mpu6050_data.accyReal  = mpu6050_data.acc[1]  * MPU6050_ACCEL_2G_SEN;
			mpu6050_data.acczReal  = mpu6050_data.acc[2]  * MPU6050_ACCEL_2G_SEN;
			
			//����Ƕ�ֵ Mahony����
			MahonyAHRSupdate(mpu6050_data.gyroxReal,mpu6050_data.gyroyReal,mpu6050_data.gyrozReal, \
							 mpu6050_data.accxReal,mpu6050_data.accyReal,mpu6050_data.acczReal,    \
							 0, 0, 0, mpu6050_data.angle);
			//����Ƕ�ֵ Madgwick����
//			MadgwickAHRSupdate(mpu6050_data.gyroxReal,mpu6050_data.gyroyReal,mpu6050_data.gyrozReal, \
//							   mpu6050_data.accxReal,mpu6050_data.accyReal,mpu6050_data.acczReal,    \
//							   0, 0, 0, mpu6050_data.angle);
			
			mpu6050_data.angleRoll  = mpu6050_data.angle[0];
			mpu6050_data.anglePitch = mpu6050_data.angle[1];
			mpu6050_data.angleYaw   = mpu6050_data.angle[2];
			
			count ++;
			if(count % 100 == 0)
			{
				printf("%f, %f, %f\r\n",mpu6050_data.anglePitch,mpu6050_data.angleRoll,mpu6050_data.angleYaw);
			}
		}
	}
}
/**
  * @brief   ʹ��DMP�����Ƕ�
  * @param    
  * @retval  void
 **/
static void cal_with_dmp()
{
	int count = 0;
	mpu_dmp_init();
	while(1)
	{
		//�ò��ָ��»�ͳ�ʼ��mpu6050ʱ��Ķ���Ĳ��������
		if(1 == data_ready)
		{
			data_ready = 0;
			if(mpu_dmp_get_data(&mpu6050_data.anglePitch,&mpu6050_data.angleRoll,&mpu6050_data.angleYaw)==0)
			{ 
				mpu6050_get_gyro(&mpu6050_data.gyro[0],&mpu6050_data.gyro[1],&mpu6050_data.gyro[2]);
				mpu6050_get_acc (&mpu6050_data.acc[0] ,&mpu6050_data.acc[1] ,&mpu6050_data.acc[2]);
				//����λ���� ����ת��
				mpu6050_data.gyroxReal = mpu6050_data.gyro[0] * MPU6050_GYRO_2000_SEN;
				mpu6050_data.gyroyReal = mpu6050_data.gyro[1] * MPU6050_GYRO_2000_SEN;
				mpu6050_data.gyrozReal = mpu6050_data.gyro[2] * MPU6050_GYRO_2000_SEN;
				mpu6050_data.accxReal  = mpu6050_data.acc[0]  * MPU6050_ACCEL_2G_SEN;
				mpu6050_data.accyReal  = mpu6050_data.acc[1]  * MPU6050_ACCEL_2G_SEN;
				mpu6050_data.acczReal  = mpu6050_data.acc[2]  * MPU6050_ACCEL_2G_SEN;
			}
			count ++;
			if(count % 100 == 0)
			{
				printf("%f, %f, %f\r\n",mpu6050_data.anglePitch,mpu6050_data.angleRoll,mpu6050_data.angleYaw);
			}
		}
	}
}



