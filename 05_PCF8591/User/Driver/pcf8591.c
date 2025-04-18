#include "pcf8591.h"

/********************User modification area begin********************/
//只需要将自己的IIC处理程序替换进去即可
#include "main.h"
#include "i2c.h"

/**
  * @brief   pcf8591的写入一个字节
  * @param   addr 设备地址   reg 寄存器地址   data 写入的数据
  * @retval  0 成功 1 失败
 **/
int pcf8591_write_one_byte(uint8_t addr,uint8_t reg,uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c1, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, &data,1,HAL_MAX_DELAY);
	return 0;
}

/**
  * @brief   pcf8591的读取一个字节数据
  * @param   addr 设备地址   reg 寄存器地址   data 读取的数据
  * @retval  0 成功 1 失败
 **/
int pcf8591_read_one_byte(uint8_t addr,uint8_t reg,uint8_t *data)
{
	HAL_I2C_Mem_Read(&hi2c1, (addr<<1), reg,I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	return 0;
}
/*********************User modification area end**************/

#define pcfAddress 0x48

void pcfAnalogWrite (int value)
{
	pcf8591_write_one_byte(pcfAddress,0x40,value);
}

uint8_t pcfAnalogRead (int pinReg)
{
	uint8_t adcx ; 

	pcf8591_read_one_byte(pcfAddress, 0x40|pinReg, &adcx);

	return adcx ;
}
