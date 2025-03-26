#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

#include "main.h"

typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

extern Kalman_t KalmanX,KalmanY;
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
 
#endif

