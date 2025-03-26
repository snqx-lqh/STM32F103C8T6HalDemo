#include "mpu6050.h"

/********************User modification area begin********************/
//只需要将自己的IIC处理程序替换进去即可
#include "main.h"
#include "i2c.h"
/**
  * @brief   MPU6050的延时函数
  * @param   xms 毫秒
  * @retval  0 成功 1 失败
 **/
void mpu6050_delay_ms(uint16_t xms)
{
	HAL_Delay(xms);
}
/**
  * @brief   MPU6050的写入一个字节
  * @param   addr 设备地址   reg 寄存器地址   data 写入的数据
  * @retval  0 成功 1 失败
 **/
int mpu6050_write_one_byte(uint8_t addr,uint8_t reg,uint8_t data)
{
	u_i2c1_write_byte(addr,reg,&data);
	return 0;
}

/**
  * @brief   MPU6050的写入多个字节
  * @param   addr 设备地址   reg 寄存器地址   data 写入的数据
  * @retval  0 成功 1 失败
 **/
int mpu6050_write_bytes(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *data)
{
	u_i2c1_write_bytes(addr, reg, data ,len);
	return 0;
}
/**
  * @brief   MPU6050的读取一个字节数据
  * @param   addr 设备地址   reg 寄存器地址   data 读取的数据
  * @retval  0 成功 1 失败
 **/
int mpu6050_read_one_byte(uint8_t addr,uint8_t reg,uint8_t *data)
{
	u_i2c1_read_byte(addr,reg,data);
	return 0;
}
/**
  * @brief   MPU6050的读取多个字节数据
  * @param   addr 设备地址   reg 寄存器地址   data 读取的数据缓存   len 读取的数据长度
  * @retval  0 成功 1 失败
 **/
int mpu6050_read_bytes(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *data)
{
	u_i2c1_read_bytes(addr,reg,data,len);
	return 0;
}

/*********************User modification area end**************/

float gyro_offset[3] = {0,0,0};

/**
  * @brief   MPU6050的初始化
  * @param   
  * @retval  0 成功 1 失败
 **/
int mpu6050_init()
{
	uint8_t res = 0;
	mpu6050_write_one_byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X80); 
    mpu6050_delay_ms(100);  
    mpu6050_write_one_byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X00); 
    mpu6050_set_gyro_fsr(GYRO_2000DPS);					        	
    mpu6050_set_acc_fsr(ACC_2G);					       	 	
    mpu6050_set_rate(100);						       	 	
    mpu6050_write_one_byte(MPU6050_ADDR,MPU_INT_EN_REG,0X01);    
    mpu6050_write_one_byte(MPU6050_ADDR,MPU_USER_CTRL_REG,0X00); 
    mpu6050_write_one_byte(MPU6050_ADDR,MPU_FIFO_EN_REG,0X00);	  
    mpu6050_write_one_byte(MPU6050_ADDR,MPU_INTBP_CFG_REG,0X9C); 
    mpu6050_read_one_byte (MPU6050_ADDR,MPU_DEVICE_ID_REG,&res); 
    if(res==MPU6050_ADDR)  
    {
        mpu6050_write_one_byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X01);  	 
        mpu6050_write_one_byte(MPU6050_ADDR,MPU_PWR_MGMT2_REG,0X00);  	 
        mpu6050_set_rate(100);						       	 
    } else return 1;
	mpu_calibration();
    return 0;
}

/**
  * @brief   设置传感器的初始化校准
  * @param    
  * @retval  设置成功与否
 **/
void mpu_calibration()
{
	int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int16_t gx, gy, gz;
    for (int i = 0; i < 100; i++)
    {
        mpu6050_get_gyro(&gx, &gy, &gz);
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        mpu6050_delay_ms(10); // 稍微延时，确保数据稳定
    }
    gyro_offset[0] = sum_gx / 100;
    gyro_offset[1] = sum_gy / 100;
    gyro_offset[2] = sum_gz / 100;
}

/**
  * @brief   设置陀螺仪传感器满量程范围
  * @param   fsr GYRO_2000DPS GYRO_1000DPS GYRO_500DPS GYRO_250DPS 
  * @retval  设置成功与否
 **/
uint8_t mpu6050_set_gyro_fsr(uint8_t fsr)
{
    return mpu6050_write_one_byte(MPU6050_ADDR,MPU_GYRO_CFG_REG,(fsr<<3)|3);//设置陀螺仪满量程范围
}

/**
  * @brief   设置加速度传感器满量程范围
  * @param   fsr ACC_16G ACC_8G ACC_4G ACC_2G 
  * @retval  设置成功与否
 **/
uint8_t mpu6050_set_acc_fsr(uint8_t fsr)
{
    return mpu6050_write_one_byte(MPU6050_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围
}

/**
  * @brief   设置数字低通滤波器
  * @param   lpf 
  * @retval  设置成功与否
 **/
uint8_t mpu6050_set_lpf(uint16_t lpf)
{
    uint8_t data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6;
    return mpu6050_write_one_byte(MPU6050_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器
}

/**
  * @brief   设置MPU6050的采样率
  * @param   rate 
  * @retval  设置成功与否
 **/
uint8_t mpu6050_set_rate(uint16_t rate)
{
    uint8_t data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=mpu6050_write_one_byte(MPU6050_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
    return mpu6050_set_lpf(rate/2);	//自动设置LPF为采样率的一半
}
/**
  * @brief   获得陀螺仪温度值
  * @param    
  * @retval  陀螺仪温度
 **/
short mpu6050_get_temperature(void)
{
    uint8_t buf[2];
    short raw;
    mpu6050_read_bytes(MPU6050_ADDR,MPU_TEMP_OUTH_REG,2,buf);
    raw=((uint16_t)buf[0]<<8)|buf[1];
    return raw;
}
/**
  * @brief   获得陀螺仪角速度值
  * @param   传入存取角速度的地址
  * @retval  0 正确读取 1 IIC读取失败 2 传入指针为空 
 **/
uint8_t mpu6050_get_gyro(int16_t *gx,int16_t *gy,int16_t *gz)
{
    uint8_t buf[6],res;
	if(gx == NULL || gy == NULL || gz == NULL)
		return 2;
    res=mpu6050_read_bytes(MPU6050_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
    if(res==0)
    {
        *gx=(((uint16_t)buf[0]<<8)|buf[1]);
        *gy=(((uint16_t)buf[2]<<8)|buf[3]);
        *gz=(((uint16_t)buf[4]<<8)|buf[5]);
		
		*gx -= gyro_offset[0];
		*gy -= gyro_offset[1];
		*gz -= gyro_offset[2];
    }
    return res;
}
/**
  * @brief   获得陀螺仪加速度计值
  * @param   传入存取加速度的地址
  * @retval  0 正确读取 1 IIC读取失败 2 传入指针为空 
 **/
uint8_t mpu6050_get_acc(int16_t *ax,int16_t *ay,int16_t *az)
{
    uint8_t buf[6],res;
	if(ax == NULL || ay == NULL || az == NULL)
		return 2;
    res=mpu6050_read_bytes(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
    if(res==0)
    {
        *ax=(((uint16_t)buf[0]<<8)|buf[1]);
        *ay=(((uint16_t)buf[2]<<8)|buf[3]);
        *az=(((uint16_t)buf[4]<<8)|buf[5]);
    }
    return res;
}

