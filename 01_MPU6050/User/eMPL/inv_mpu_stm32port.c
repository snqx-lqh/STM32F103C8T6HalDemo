#include "inv_mpu_stm32port.h"
#include <math.h>

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "stdio.h"

#define ERROR_MPU_INIT      -1
#define ERROR_SET_SENSOR    -2
#define ERROR_CONFIG_FIFO   -3
#define ERROR_SET_RATE      -4
#define ERROR_LOAD_MOTION_DRIVER    -5 
#define ERROR_SET_ORIENTATION       -6
#define ERROR_ENABLE_FEATURE        -7
#define ERROR_SET_FIFO_RATE         -8
#define ERROR_SELF_TEST             -9
#define ERROR_DMP_STATE             -10

#define DEFAULT_MPU_HZ  100
#define Q30  1073741824.0f

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
    int ret;
    struct int_param_s int_param;

    ret = mpu_init(&int_param);
    if(ret != 0)return ERROR_MPU_INIT;

    //���ô�����
    ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if(ret != 0)return ERROR_SET_SENSOR;
    
    //����fifo
    ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if(ret != 0)return ERROR_CONFIG_FIFO;
    
    //���ò�����
    ret = mpu_set_sample_rate(DEFAULT_MPU_HZ);
    if(ret != 0)return ERROR_SET_RATE;
    
    //����DMP�̼�
    ret = dmp_load_motion_driver_firmware();
    if(ret != 0)return ERROR_LOAD_MOTION_DRIVER;
    
    //���������Ƿ���
    ret = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    if(ret != 0)return ERROR_SET_ORIENTATION;
    
    //����DMP����
    ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | 
            DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    if(ret != 0)return ERROR_ENABLE_FEATURE;
    
    //�����������
    ret = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    if(ret != 0)return ERROR_SET_FIFO_RATE;
    
    //�Լ�
    ret = run_self_test();
    if(ret != 0)return ERROR_SELF_TEST;
    
    //ʹ��DMP
    ret = mpu_set_dmp_state(1);
    if(ret != 0)return ERROR_DMP_STATE;
    

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