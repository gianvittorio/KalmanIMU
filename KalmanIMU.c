#include "KalmanIMU.h"

float norm(float x, float y, float z)
{
	return sqrt(x*x + y*y + z*z);
}

void calcEulerAngles(float * angles, float accX, float accY, float accZ, float magX, float magY, float magZ)
{
	/*accX = accX/norm(accX, accY, accZ);
	accY = accX/norm(accX, accY, accZ);
	accZ = accX/norm(accX, accY, accZ); */
	/*magX = magX/norm(magX, magY, magZ);
	magY = magY/norm(magX, magY, magZ);
	magZ = magZ/norm(magX, magY, magZ);*/

	*angles = atan2(accY, accZ) * RAD_TO_DEG;
	*(angles + 1) = atan(-accX / sqrt(accY*accY + accZ * accZ)) * RAD_TO_DEG;
	float Mx = magX * cos(angles[1]) + magZ * sin(angles[1]);
	float My = magX * sin(angles[0])*sin(angles[1]) - magY * cos(angles[0]) - magZ * sin(angles[0])*cos(angles[1]);
	*(angles + 2) = atan2(-My, Mx) * RAD_TO_DEG;
}

void KalmanInit(Kalman * K, float angle)
{
	/* We will set the variables like so, these can also be tuned by the user */
    K->Q_angle = Q_ANGLE;
    K->Q_bias = Q_BIAS;
    K->R_measure = R_MEASURE;

    K->angle = angle; // Reset the angle
    K->bias = 0.0f; // Reset bias

    K->P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    K->P[0][1] = 0.0f;
    K->P[1][0] = 0.0f;
	K->P[1][1] = 0.0f;
}

void KalmanStep(Kalman * K, float newAngle, float newRate)
{
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	// Modified by Kristian Lauszus
	// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	K->rate = newRate - K->bias;
	K->angle += dt * K->rate;
	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	K->P[0][0] += dt * (dt*K->P[1][1] - K->P[0][1] - K->P[1][0]) + K->Q_angle;
	K->P[0][1] -= dt * K->P[1][1];
	K->P[1][0] -= dt * K->P[1][1];
	K->P[1][1] += K->Q_bias;
	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	float S = K->P[0][0] + K->R_measure; // Estimate error
	/* Step 5 */
	float G[2]; // Kalman gain - This is a 2x1 vector
	G[0] = K->P[0][0] / S;
	G[1] = K->P[1][0] / S;
	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	float y = newAngle - K->angle; // Angle difference
	/* Step 6 */
	K->angle += G[0] * y;
	K->bias += G[1] * y;

	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	float P00_temp = K->P[0][0];
	float P01_temp = K->P[0][1];

	K->P[0][0] -= G[0] * P00_temp;
	K->P[0][1] -= G[0] * P01_temp;
	K->P[1][0] -= G[1] * P00_temp;
	K->P[1][1] -= G[1] * P01_temp;
}
