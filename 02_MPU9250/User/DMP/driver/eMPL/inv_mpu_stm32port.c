#include "inv_mpu_stm32port.h"

#include <math.h>
#include "inv_mpu.h"
#include "mpl.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "data_builder.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "stdio.h"


#define DEFAULT_MPU_HZ  (100)
#define COMPASS_READ_MS (100)
#define Q30  1073741824.0f
#define Q16  65536.0f

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
/* (ʹ��Ai���׷�����һ��ԭע��)
 * �������������κη���װ�����ϡ�
 * ����İ�װ�������MPL��δ�����������תԭʼ���ݡ�
 * TODO: ����ľ���ָ����Invensense�ڲ����԰��ϵ����á�
 * �����Ҫ�����޸ľ�����ƥ�����ض����õ�оƬ���������
*/
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};
//�����Ʒ�������
static signed char comp_orientation[9] = { 0, 1, 0,
                                           1, 0, 0,
                                           0, 0,-1};
/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
/* (ʹ��Ai���׷�����һ��ԭע��)
 * ����������������������󣨲μ�gyro_orientation��ת��Ϊ������ʾ���Թ�DMPʹ�á�
 * ע�ͣ���Щ�����Ǵ�Invensense��MPL���õġ�
*/
static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;
    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}

/**
  * @brief   �Լ����
  * @param    
  * @retval  void
 **/
static int run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
	//result = mpu_run_6500_self_test(gyro, accel,0);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    } else {
        return -1;
    }

    return 0;
}

/**
  * @brief   ��ʼ��MPU6050��DMP�������
  * @param    
  * @retval  void
 **/
int mpu_dmp_init(void)
{
	uint8_t res=0;
    struct int_param_s int_param;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    unsigned short compass_fsr;
    
	if(mpu_init(&int_param)==0)	//��ʼ��MPU9250
	{	 
        res=inv_init_mpl();     //��ʼ��MPL
        if(res)return 1;
        inv_enable_quaternion();
        inv_enable_9x_sensor_fusion();
        inv_enable_fast_nomot();
        inv_enable_gyro_tc();
        inv_enable_vector_compass_cal();
        inv_enable_magnetic_disturbance();
        inv_enable_eMPL_outputs();
        res=inv_start_mpl();    //����MPL
        if(res)return 1;
		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL|INV_XYZ_COMPASS);//��������Ҫ�Ĵ�����
		if(res)return 2; 
		res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);   //����FIFO
		if(res)return 3; 
		res=mpu_set_sample_rate(DEFAULT_MPU_HZ);	            //���ò�����
		if(res)return 4; 
        res=mpu_set_compass_sample_rate(1000/COMPASS_READ_MS);  //���ô����Ʋ�����
        if(res)return 5;
        mpu_get_sample_rate(&gyro_rate);
        mpu_get_gyro_fsr(&gyro_fsr);
        mpu_get_accel_fsr(&accel_fsr);
        mpu_get_compass_fsr(&compass_fsr);
        inv_set_gyro_sample_rate(1000000L/gyro_rate);
        inv_set_accel_sample_rate(1000000L/gyro_rate);
        inv_set_compass_sample_rate(COMPASS_READ_MS*1000L);
        inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_orientation),(long)gyro_fsr<<15);
        inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_orientation),(long)accel_fsr<<15);
        inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(comp_orientation),(long)compass_fsr<<15);
            
            
		res=dmp_load_motion_driver_firmware();		             //����dmp�̼�
		if(res)return 6; 
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//���������Ƿ���
		if(res)return 7; 
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	            //����dmp����
		    DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
		    DMP_FEATURE_GYRO_CAL);
		if(res)return 8; 
		res=dmp_set_fifo_rate(DEFAULT_MPU_HZ);	//����DMP�������(��󲻳���200Hz)
		if(res)return 9;   
		res=run_self_test();		//�Լ�
		if(res)return 10;    
		res=mpu_set_dmp_state(1);	//ʹ��DMP
		if(res)return 11;     
	}
	return 0;
}

/**
  * @brief   ��ȡ��Ԫ��ֵ������õ�ʵ�ʵĽǶ�ֵ
  * @param    
  * @retval  void
 **/
int mpu_dmp_get_data(float *pitch, float *roll, float *yaw)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    short gyro[3];
    short accel[3];
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    if(dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more))
    {
        return -1;
    }

    if(sensors & INV_WXYZ_QUAT)
    {
        q0 = quat[0] / Q30;
        q1 = quat[1] / Q30;
        q2 = quat[2] / Q30;
        q3 = quat[3] / Q30;

        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3; // pitch
        *roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // roll
        *yaw = atan2(2 * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3; // yaw
    }

    return 0;
}

int mpu_mpl_get_data(float *pitch,float *roll,float *yaw)
{
	unsigned long sensor_timestamp;
	short gyro[3], accel_short[3],compass_short[3],sensors;
	unsigned char more;
	long compass[3],accel[3],quat[4],temperature; 
    long data[9];
	inv_time_t timestamp;
    int8_t accuracy;
    
	if(dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors,&more))return 1;	 

    if(sensors&INV_XYZ_GYRO)
    {
        inv_build_gyro(gyro,sensor_timestamp);          //�������ݷ��͸�MPL
        mpu_get_temperature(&temperature,&sensor_timestamp);
        inv_build_temp(temperature,sensor_timestamp);   //���¶�ֵ����MPL��ֻ����������Ҫ�¶�ֵ
    }
    
    if(sensors&INV_XYZ_ACCEL)
    {
        accel[0] = (long)accel_short[0];
        accel[1] = (long)accel_short[1];
        accel[2] = (long)accel_short[2];
        inv_build_accel(accel,0,sensor_timestamp);      //�Ѽ��ٶ�ֵ����MPL
    }
    
    if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) 
    {
        compass[0]=(long)compass_short[0];
        compass[1]=(long)compass_short[1];
        compass[2]=(long)compass_short[2];
        inv_build_compass(compass,0,sensor_timestamp); //�Ѵ�����ֵ����MPL
    }
    inv_execute_on_data();
    inv_get_sensor_type_euler(data,&accuracy,&timestamp);
    
    *roll  = (data[0]/Q16);
    *pitch = -(data[1]/Q16);
    *yaw   = -data[2] / Q16;
	return 0;
}
