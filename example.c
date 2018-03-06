#include "example.h"

#define N 100
#define M 1000
#define E 3

void runExample()
{
	FILE *fp = NULL;
	char * str = (char *)malloc(N * sizeof(char));

	int SampleNumber[M];
	float MagX[M], MagY[M], MagZ[M],
		AccX[M], AccY[M], AccZ[M],
		GyroR[M], GyroP[M], GyroY[M],
		LButton[M], RButton[M];

	int k = 0;
	fp = fopen("Pickup.txt", "r");
	while (fgets(str, N, fp) != NULL)
	{
		if ((*str != '%') && (*str != '\n'))
		{
			sscanf(str, "%d %f %f %f %f %f %f %f %f %f %f %f",
				(SampleNumber + k),
				(MagX + k), (MagY + k), (MagZ + k),
				(AccX + k), (AccY + k), (AccZ + k),
				(GyroR + k), (GyroP + k), (GyroY + k),
				(LButton + k), (RButton + k));

			/* Since the file ha a column for line count, check if it matches our own count, so we can exit the loop: */
			if (*(SampleNumber + k) != (k + 1))
				break;
			++k;
		}
	}

	free(str);
	fclose(fp);

	FILE * fp2 = fopen("results.txt", "w");
	Kalman K[3];
	float angles[] = { 0.0f, 0.0f, 0.0f };
	calcEulerAngles(angles,
		*AccX, *AccY, *AccZ,
		*MagX, *MagY, *MagZ);
	KalmanInit(K, *angles);
	KalmanInit(K + 1, *(angles + 1));
	KalmanInit(K + 2, *(angles + 2));
	fprintf(fp2, "%f %f %f %f %f %f %f %f %f\n", K[0].angle, K[1].angle, K[2].angle, *angles, angles[1], angles[2],
		*GyroR, *GyroP, *GyroY);
	for (int i = 1; i < k; ++i)
	{
		calcEulerAngles(angles,
			AccX[i], AccY[i], AccZ[i],
			MagX[i], MagY[i], MagZ[i]);

		KalmanStep(K, *angles, *(GyroR + i));
		KalmanStep(K + 1, *(angles + 1), *(GyroP + i));
		KalmanStep(K + 2, *(angles + 2), *(GyroY + i));
		fprintf(fp2, "%f %f %f %f %f %f %f %f %f\n", K[0].angle, K[1].angle, K[2].angle, *angles, angles[1], angles[2],
			*(GyroR + i), *(GyroP + i), *(GyroY + i));
	}

	fclose(fp2);
}