#include "dev_mpu_mtx.h"
#include "dev_mpu_mtx_inv.h"
#include <math.h>
#include <tgmath.h>

/* Inverts a matrix using the Gauss Jordan method.
 * returns 0 on success, 1 on failure.
 */
#define MTX_INV									\
int i, j;									\
for (i = 0; i < n; i++) {							\
	for (j = 0; j < n; j++) {						\
		c[i][j] = 0;							\
		b[i][j] = a[i][j];						\
	}									\
	c[i][i] = 1;								\
}										\
int iPass, imx, icol, irow;							\
for (iPass = 0; iPass < n; iPass++) {						\
	imx = iPass;								\
	for (irow = iPass; irow < n; irow++) {					\
		if (fabs(b[irow][iPass]) > fabs(b[imx][iPass])) {		\
			imx = irow;						\
		}								\
	}									\
	if (imx != iPass) {							\
		for (icol = 0; icol < n; icol++) {				\
			temp = c[iPass][icol];					\
			c[iPass][icol] = c[imx][icol];				\
			c[imx][icol] = temp;					\
			if (icol >= iPass) {					\
				temp = b[iPass][icol];				\
				b[iPass][icol] = b[imx][icol];			\
				b[imx][icol] = temp;				\
			}							\
		}								\
	}									\
	pivot = b[iPass][iPass];						\
	det  *= b[iPass][iPass];						\
	if (det == 0) {								\
		return -1;							\
	}									\
	for (icol = 0; icol < n; icol++) {					\
		c[iPass][icol] /=  pivot;					\
		if (icol >= iPass) {						\
			b[iPass][icol] /= pivot;				\
		}								\
	}									\
	for (irow = 0; irow < n; irow++) {					\
		if (irow != iPass) {						\
			factor = b[irow][iPass];				\
		}								\
		for (icol = 0; icol < n; icol++){				\
			if (irow != iPass){					\
				c[irow][icol] -= factor * c[iPass][icol];	\
				b[irow][icol] -= factor * b[iPass][icol];	\
			}							\
		}								\
	}									\
}										\
return 1;

int mtxf_inv(float * A, float * B, int n, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*b)[n] = (float (*)[n])B;
	float (*c)[n] = (float (*)[n])C;

	float det = 1;
	float temp, pivot, factor;
	MTX_INV;
}
int mtxlf_inv(double * A, double * B, int n, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*b)[n] = (double (*)[n])B;
	double (*c)[n] = (double (*)[n])C;

	double det = 1;
	double temp, pivot, factor;
	MTX_INV;
}
int mtxLf_inv(long double * A, long double * B, int n, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*b)[n] = (long double (*)[n])B;
	long double (*c)[n] = (long double (*)[n])C;

	long double det = 1;
	long double temp, pivot, factor;
	MTX_INV;
}

