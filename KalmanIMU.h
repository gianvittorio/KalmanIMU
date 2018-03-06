#ifndef KALMANIMU_H
#define KALMANIMU_H

#include <math.h>
#include <stdio.h>

#define Q_ANGLE 1e-02
#define Q_BIAS 1e-02
#define R_MEASURE 1e-02
#define dt 0.0079
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

/**
 * @brief Calculates roll, pitch and yaw angles gotten from 9DOF measuremens
 * 
 * @param angles float pointer which is going to store roll, pitch and yaw
 * @param accX X-axis measurement from accelerometer
 * @param accY Y-axis measurement from accelerometer
 * @param accZ Z-axis measurement from accelerometer
 * @param magX X-axis measurement from magnetometer
 * @param magY Y-axis measurement from magnetometer
 * @param magZ Z-axis measurement from magnetometer
 */
void calcEulerAngles(float * angles, float accX, float accY, float accZ, float magX, float magY, float magZ);

/**
 * @brief Initializes Kalman structs with according parameters
 * 
 * @param K K pointer which stores Kalman Filter parameters
 * @param angle current angle, calculated from acc and mag measurements
 */
void KalmanInit(Kalman * K, float angle);

/**
 * @brief Runs one step of Kalman Filter algorithm
 * 
 * @param K Kalman struct pointer
 * @param newAngle current angle, calculated from acc and mag measurements
 * @param newRate current angular rate, calculated from gyro
 */
void KalmanStep(Kalman * K, float newAngle, float newRate);

#endif