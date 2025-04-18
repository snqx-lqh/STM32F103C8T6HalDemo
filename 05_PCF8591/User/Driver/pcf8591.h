#ifndef _PCF8591_H
#define _PCF8591_H

#include "main.h"

void    pcfAnalogWrite (int value);
uint8_t pcfAnalogRead (int pinReg);

#endif
