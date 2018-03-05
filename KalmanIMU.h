#ifndef KALMANIMU_H
#define KALMANIMU_H

#include <math.h>
#include <stdio.h>

#define Q_ANGLE 0.01f
#define Q_BIAS 0.03f
#define R_MEASURE 0.03f
#define dt 0.0078829f
#define RAD_TO_DEG 180/M_PI

typedef struct
{
	/* Kalman filter variables */
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
} Kalman;


void calcEulerAngles(float * angles, float accX, float accY, float accZ, float magX, float magY, float magZ);

void KalmanInit(Kalman * K, float angle);

void KalmanStep(Kalman * K, float newAngle, float newRate);

#endif