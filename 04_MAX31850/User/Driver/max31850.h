#ifndef _MAX_31850_H
#define _MAX_31850_H

#include "main.h"

// 初始化max31850，包含引脚初始化
void max31850_init(void);

// 读取max31850的温度，带扫描地址
int get_max31850_temp(float *celsius,float *fahrenheit);

// 读取max31850的温度，跳过地址扫描
int get_max31850_temp_skiprom(float *celsius,float *fahrenheit);
#endif
