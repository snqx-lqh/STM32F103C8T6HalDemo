#include "FirstOrderLowPassFilter.h"

FOLPF_t FOLPF_anglex= {
    .weight = 0.02f
};

FOLPF_t FOLPF_angley= {
    .weight = 0.02f
};

void FirstOrderLowPassFilter(FOLPF_t *FOLPF_angle,float angle_m, float gyro_m, float dt)
{
    FOLPF_angle->angle = FOLPF_angle->weight * angle_m+ (1 - FOLPF_angle->weight) * (FOLPF_angle->angle + gyro_m * dt);
}

