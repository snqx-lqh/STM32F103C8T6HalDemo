#ifndef _DELAY_H
#define _DELAY_H

#include "main.h"

void delay_init(void);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);

void DWT_Delay_Init(void);
void DWT_Delay_us(uint32_t us);
void DWT_Delay_ms(uint32_t ms);

#endif
