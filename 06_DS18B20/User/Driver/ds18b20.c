#include "ds18b20.h"

#include "u_one_wire.h"
#include "delay.h"
#include "stdio.h"

/***************    用户处理区域    ****************/

#define ds18b20_delay_ms DWT_Delay_ms

static uint8_t gpio_init(void)
{
	return 0;
}

static uint8_t set_pin_dir(one_wire_dir_t one_wire_dir)
{
	if(one_wire_dir == ONE_WIRE_DIR_IN)
	{
		GPIOA->CRL&=0XFFFFFFF0;GPIOA->CRL|=8<<(4*0);
	}else if(one_wire_dir == ONE_WIRE_DIR_OUT)
	{
		GPIOA->CRL&=0XFFFFFFF0;GPIOA->CRL|=3<<(4*0);
	}
	return 0;
}

static  uint8_t set_pin_level(uint8_t level)
{
	if(0 == level)
	{
		GPIOA->BRR  = GPIO_PIN_0;
	}else if(1 == level)
	{
		GPIOA->BSRR = GPIO_PIN_0;
	}
	return 0;
}

static uint8_t read_pin_level(void)
{
	uint8_t read_pin ;
	read_pin = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
	return read_pin;
}

one_wire_dev_t ds18b20={
	.init = gpio_init,
	.set_pin_dir = set_pin_dir,
	.set_pin_level= set_pin_level,
	.read_pin_level= read_pin_level,
};

/**********************************************************/

#define TYPE_DS18S20 0
#define TYPE_DS18B20 1
#define TYPE_DS18S22 2
#define TYPE_MAX31850 3

/**
  * @brief   初始化ds18b20，包含引脚初始化
  * @param   
  * @retval   
 **/
void ds18b20_init()
{
	 one_wire_begin(&ds18b20);
}

uint8_t present = 0;
uint8_t temptype;
uint8_t one_wire_data[12];
uint8_t one_wire_addr[8];

/**
  * @brief   读取ds18b20的温度，带扫描地址
  * @param   celsius：摄氏度 fahrenheit：华氏温度
  * @retval  
 **/
int get_ds18b20_temp(float *celsius,float *fahrenheit)
{
    uint8_t i;
	
    if(celsius == NULL || fahrenheit == NULL)
        return -1;

    if ( !one_wire_search(&ds18b20,one_wire_addr,true)) 
	{
        one_wire_reset_search(&ds18b20);
        ds18b20_delay_ms(250);
        return -2;
    }

    if (crc8(one_wire_addr, 7) != one_wire_addr[7]) {
        return -3;
    }

    // the first ROM byte indicates which chip
    switch (one_wire_addr[0]) {
    case 0x10:
        temptype = TYPE_DS18S20;
        break;
    case 0x28:
        temptype = TYPE_DS18B20;
        break;
    case 0x22:
        temptype = TYPE_DS18S22;
        break;
    case 0x3B:
        temptype = TYPE_MAX31850;
        break;
    default:
        return -4;
    }

    one_wire_reset(&ds18b20);
    one_wire_select(&ds18b20,one_wire_addr);
    one_wire_write(&ds18b20,0x44, 1);        // start conversion, with parasite power on at the end

    ds18b20_delay_ms(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    present = one_wire_reset(&ds18b20);
    one_wire_select(&ds18b20,one_wire_addr);
    one_wire_write(&ds18b20,0xBE,0);         // Read Scratchpad

    for ( i = 0; i < 9; i++) {           // we need 9 bytes
        one_wire_read(&ds18b20,&one_wire_data[i]);
    }

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (one_wire_data[1] << 8) | one_wire_data[0];
    if (temptype == TYPE_DS18S20) {
        raw = raw << 3; // 9 bit resolution default
        if (one_wire_data[7] == 0x10) {
            // "count remain" gives full 12 bit resolution
            raw = (raw & 0xFFF0) + 12 - one_wire_data[6];
        }
    } else if (temptype == TYPE_MAX31850) {
        printf("--------------------------------\r\n");
        if (raw & 0x01) {
            return -4;
        }
    } else {
        uint8_t cfg = (one_wire_data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
    }
    *celsius = (float)raw / 16.0;
    *fahrenheit = *celsius * 1.8 + 32.0;
	return 0;
}

/**
  * @brief   读取ds18b20的温度，跳过地址扫描
  * @param   celsius：摄氏度 fahrenheit：华氏温度
  * @retval  
 **/
int get_ds18b20_temp_skiprom(float *celsius,float *fahrenheit)
{
    uint8_t i;
 
    if(celsius == NULL || fahrenheit == NULL)
        return -1;
	
    one_wire_reset(&ds18b20);
    one_wire_skip(&ds18b20);
    one_wire_write(&ds18b20,0x44, 1);        // start conversion, with parasite power on at the end

    ds18b20_delay_ms(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    present = one_wire_reset(&ds18b20);
    one_wire_skip(&ds18b20);
    one_wire_write(&ds18b20,0xBE,0);         // Read Scratchpad
    for ( i = 0; i < 9; i++) {                // we need 9 bytes
        one_wire_read(&ds18b20,&one_wire_data[i]);
    }
    
	int16_t raw = (one_wire_data[1] << 8) | one_wire_data[0];
	
	uint8_t cfg = (one_wire_data[4] & 0x60);
	// at lower res, the low bits are undefined, so let's zero them
	if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
	else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
	else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    
	*celsius = (float)raw / 16.0;
    *fahrenheit = *celsius * 1.8 + 32.0;
	return 0;
}
