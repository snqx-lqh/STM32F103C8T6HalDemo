#ifndef _MAX_31850_H
#define _MAX_31850_H

#include "main.h"

// ��ʼ��max31850���������ų�ʼ��
void max31850_init(void);

// ��ȡmax31850���¶ȣ���ɨ���ַ
int get_max31850_temp(float *celsius,float *fahrenheit);

// ��ȡmax31850���¶ȣ�������ַɨ��
int get_max31850_temp_skiprom(float *celsius,float *fahrenheit);
#endif
