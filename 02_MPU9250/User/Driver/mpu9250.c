#include "mpu9250.h"

/********************User modification area begin********************/
//ֻ��Ҫ���Լ���IIC��������滻��ȥ����
#include "main.h"
#include "i2c.h"
/**
  * @brief   MPU9250����ʱ����
  * @param   xms ����
  * @retval  0 �ɹ� 1 ʧ��
 **/
void mpu9250_delay_ms(uint16_t xms)
{
	HAL_Delay(xms);
}
/**
  * @brief   MPU9250��д��һ���ֽ�
  * @param   addr �豸��ַ   reg �Ĵ�����ַ   data д�������
  * @retval  0 �ɹ� 1 ʧ��
 **/
int mpu9250_write_one_byte(uint8_t addr,uint8_t reg,uint8_t data)
{
	u_i2c1_write_byte(addr,reg,&data);
	return 0;
}

/**
  * @brief   MPU9250��д�����ֽ�
  * @param   addr �豸��ַ   reg �Ĵ�����ַ   data д�������
  * @retval  0 �ɹ� 1 ʧ��
 **/
int mpu9250_write_bytes(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *data)
{
	u_i2c1_write_bytes(addr, reg, data ,len);
	return 0;
}
/**
  * @brief   MPU9250�Ķ�ȡһ���ֽ�����
  * @param   addr �豸��ַ   reg �Ĵ�����ַ   data ��ȡ������
  * @retval  0 �ɹ� 1 ʧ��
 **/
int mpu9250_read_one_byte(uint8_t addr,uint8_t reg,uint8_t *data)
{
	u_i2c1_read_byte(addr,reg,data);
	return 0;
}
/**
  * @brief   MPU9250�Ķ�ȡ����ֽ�����
  * @param   addr �豸��ַ   reg �Ĵ�����ַ   data ��ȡ�����ݻ���   len ��ȡ�����ݳ���
  * @retval  0 �ɹ� 1 ʧ��
 **/
int mpu9250_read_bytes(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *data)
{
	u_i2c1_read_bytes(addr,reg,data,len);
	return 0;
}
/*********************User modification area end**************/
float gyro_offset[3] = {0,0,0};
/**
  * @brief   MPU9250�ĳ�ʼ��
  * @param   
  * @retval  0 �ɹ� 1 ʧ��
 **/
int mpu9250_init()
{
	uint8_t res = 0;
	
    mpu9250_write_one_byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//��λMPU9250
    mpu9250_delay_ms(100);  //��ʱ100ms
    mpu9250_write_one_byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//����MPU9250
    mpu9250_set_gyro_fsr(GYRO_2000DPS);					        	
    mpu9250_set_acc_fsr(ACC_2G);					       	 	
    mpu9250_set_rate(100);	
    mpu9250_write_one_byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   // 
	mpu9250_write_one_byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C��ģʽ�ر�
	mpu9250_write_one_byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	mpu9250_write_one_byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//
    mpu9250_read_one_byte (MPU9250_ADDR,MPU_DEVICE_ID_REG,&res); 
    if(res==MPU6500_ID1||res==MPU6500_ID2) 				//����ID��ȷ
    {
        mpu9250_write_one_byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//����CLKSEL,PLL X��Ϊ�ο�
        mpu9250_write_one_byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//���ٶ��������Ƕ�����
		mpu9250_set_rate(100);						       			//���ò�����Ϊ100Hz   
    }else return 1;
	
	mpu9250_read_one_byte (AK8963_ADDR,MAG_WIA,&res); //��ȡAK8963 ID   		
    if(res==AK8963_ID)
    {
        mpu9250_write_one_byte(AK8963_ADDR,MAG_CNTL2,0X01);		//��λAK8963
		mpu9250_delay_ms(50);
        mpu9250_write_one_byte(AK8963_ADDR,MAG_CNTL1,0X11);		//����AK8963Ϊ���β���
    }else return 1;
	
	mpu_calibration();
    
	return 0;
}

/**
  * @brief   ���ô������ĳ�ʼ��У׼
  * @param    
  * @retval  ���óɹ����
 **/
void mpu_calibration()
{
	int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int16_t gx, gy, gz;
    for (int i = 0; i < 100; i++)
    {
        mpu9250_get_gyro(&gx, &gy, &gz);
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        mpu9250_delay_ms(10); // ��΢��ʱ��ȷ�������ȶ�
    }
    gyro_offset[0] = sum_gx / 100;
    gyro_offset[1] = sum_gy / 100;
    gyro_offset[2] = sum_gz / 100;
}

/**
  * @brief   ���������Ǵ����������̷�Χ
  * @param   fsr GYRO_2000DPS GYRO_1000DPS GYRO_500DPS GYRO_250DPS 
  * @retval  ���óɹ����
 **/
uint8_t mpu9250_set_gyro_fsr(uint8_t fsr)
{
    return mpu9250_write_one_byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,(fsr<<3)|3);//���������������̷�Χ
}

/**
  * @brief   ���ü��ٶȴ����������̷�Χ
  * @param   fsr ACC_16G ACC_8G ACC_4G ACC_2G 
  * @retval  ���óɹ����
 **/
uint8_t mpu9250_set_acc_fsr(uint8_t fsr)
{
    return mpu9250_write_one_byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ
}

/**
  * @brief   �������ֵ�ͨ�˲���
  * @param   lpf 
  * @retval  ���óɹ����
 **/
uint8_t mpu9250_set_lpf(uint16_t lpf)
{
    uint8_t data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6;
    return mpu9250_write_one_byte(MPU9250_ADDR,MPU_CFG_REG,data);//�������ֵ�ͨ�˲���
}

/**
  * @brief   ����MPU9250�Ĳ�����
  * @param   rate 
  * @retval  ���óɹ����
 **/
uint8_t mpu9250_set_rate(uint16_t rate)
{
    uint8_t data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=mpu9250_write_one_byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
    return mpu9250_set_lpf(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

/**
  * @brief   ��������ǽ��ٶ�ֵ
  * @param   �����ȡ���ٶȵĵ�ַ
  * @retval  0 ��ȷ��ȡ 1 IIC��ȡʧ�� 2 ����ָ��Ϊ�� 
 **/
uint8_t mpu9250_get_gyro(int16_t *gx,int16_t *gy,int16_t *gz)
{
    uint8_t buf[6],res;
	if(gx == NULL || gy == NULL || gz == NULL)
		return 2;
    res=mpu9250_read_bytes(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
    if(res==0)
    {
        *gx=(int16_t)(((uint16_t)buf[0]<<8)|buf[1]);
        *gy=(int16_t)(((uint16_t)buf[2]<<8)|buf[3]);
        *gz=(int16_t)(((uint16_t)buf[4]<<8)|buf[5]);
    }
    return res;
}
/**
  * @brief   ��������Ǽ��ٶȼ�ֵ
  * @param   �����ȡ���ٶȵĵ�ַ
  * @retval  0 ��ȷ��ȡ 1 IIC��ȡʧ�� 2 ����ָ��Ϊ�� 
 **/
uint8_t mpu9250_get_acc(int16_t *ax,int16_t *ay,int16_t *az)
{
    uint8_t buf[6],res;
	if(ax == NULL || ay == NULL || az == NULL)
		return 2;
    res=mpu9250_read_bytes(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
    if(res==0)
    {
        *ax=(int16_t)(((uint16_t)buf[0]<<8)|buf[1]);
        *ay=(int16_t)(((uint16_t)buf[2]<<8)|buf[3]);
        *az=(int16_t)(((uint16_t)buf[4]<<8)|buf[5]);
    }
    return res;
}

/**
  * @brief   ��ô����Ƽ�ֵ
  * @param    
  * @retval  0 ��ȷ��ȡ 1 IIC��ȡʧ�� 2 ����ָ��Ϊ�� 
 **/
uint8_t mpu9250_get_mag(int16_t *mx,int16_t *my,int16_t *mz)
{
    uint8_t buf[6],res;
	if(mx == NULL || my == NULL || mz == NULL)
		return 2;
    res=mpu9250_read_bytes(AK8963_ADDR,MAG_XOUT_L,6,buf);
    if(res==0)
    {
        *mx=(int16_t)(((uint16_t)buf[1]<<8)|buf[0]);
        *my=(int16_t)(((uint16_t)buf[3]<<8)|buf[2]);
        *mz=(int16_t)(((uint16_t)buf[5]<<8)|buf[4]);
    }
	mpu9250_write_one_byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ
    return res;
}

