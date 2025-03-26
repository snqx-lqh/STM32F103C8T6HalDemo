#ifndef _FIRST_ORDER_LOW_PASS_FILTER_H
#define _FIRST_ORDER_LOW_PASS_FILTER_H

#include "main.h"

typedef struct {
    float angle;
    float weight;
}FOLPF_t;

extern FOLPF_t FOLPF_anglex;
extern FOLPF_t FOLPF_angley;

void FirstOrderLowPassFilter(FOLPF_t *FOLPF_angle,float angle_m, float gyro_m, float dt);

#endif
