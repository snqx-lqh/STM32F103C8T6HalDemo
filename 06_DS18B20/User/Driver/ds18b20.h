#ifndef _DS18B20_H
#define _DS18B20_H

#include "main.h"

void ds18b20_init(void);
int get_ds18b20_temp(float *celsius,float *fahrenheit);
int get_ds18b20_temp_skiprom(float *celsius,float *fahrenheit);

#endif
