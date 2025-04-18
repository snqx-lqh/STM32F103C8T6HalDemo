#include "pcf8591.h"

/********************User modification area begin********************/
//ֻ��Ҫ���Լ���IIC��������滻��ȥ����
#include "main.h"
#include "i2c.h"

/**
  * @brief   pcf8591��д��һ���ֽ�
  * @param   addr �豸��ַ   reg �Ĵ�����ַ   data д�������
  * @retval  0 �ɹ� 1 ʧ��
 **/
int pcf8591_write_one_byte(uint8_t addr,uint8_t reg,uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c1, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, &data,1,HAL_MAX_DELAY);
	return 0;
}

/**
  * @brief   pcf8591�Ķ�ȡһ���ֽ�����
  * @param   addr �豸��ַ   reg �Ĵ�����ַ   data ��ȡ������
  * @retval  0 �ɹ� 1 ʧ��
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
