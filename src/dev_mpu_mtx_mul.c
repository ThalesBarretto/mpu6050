#include "dev_mpu_mtx.h"

#define MTX_MUL								\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			for (int k = 0; k < p; k++) {			\
				c[i][j] += (a[i][k] * b[k][j]);		\
			}						\
		}							\
	}

/* C(mxn) = A(mxp) * B(pxn) */
void mtxi_mul(const int * const A, const int * const B, const int m, const int p, const int n, int *C)
{
	const int (*a)[p] = (const int (*)[p])A;
	const int (*b)[n] = (const int (*)[n])B;
	int (*c)[n] = (int (*)[n])C;
	MTX_MUL;
}
void mtxf_mul(const float * A, const float * B, const int m, const int p, const int n, float *C)
{
	const float (*a)[p] = (const float (*)[p])A;
	const float (*b)[n] = (const float (*)[n])B;
	float (*c)[n] = (float (*)[n])C;
	MTX_MUL;
}
void mtxlf_mul(const double * A, const double * B, const int m, const int p, const int n, double *C)
{
	const double (*a)[p] = (const double (*)[p])A;
	const double (*b)[n] = (const double (*)[n])B;
	double (*c)[n] = (double (*)[n])C;
	MTX_MUL;
}
void mtxLf_mul(const long double * A, const long double * B, const int m, const int p, const int n, long double *C)
{
	const long double (*a)[p] = (const long double (*)[p])A;
	const long double (*b)[n] = (const long double (*)[n])B;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_MUL;
}
