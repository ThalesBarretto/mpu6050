#include "dev_mpu_mtx.h"

#define MTX_ZER								\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = 0;					\
		}							\
	}

#define MTX_EYE								\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = 0;					\
		}							\
		c[i][i] = 1;						\
	}

#define MTX_CPY								\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = a[i][j];				\
		}							\
	}

#define MTX_TRN								\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = a[j][i];				\
		}							\
	}

#define MTX_ADD								\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = a[i][j] + b[i][j];			\
		}							\
	}

#define MTX_SUB								\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = a[i][j] - b[i][j];			\
		}							\
	}

#define MTX_MUL								\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = 0;					\
			for (int k = 0; k < p; k++) {			\
				c[i][j] += (a[i][k] * b[k][j]);		\
			}						\
		}							\
	}

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

#define MTX_ADD_SCL							\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = a[i][j] + k;					\
		}							\
	}

#define MTX_SUB_SCL							\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = a[i][j] - k;					\
		}							\
	}

#define MTX_MUL_SCL							\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = a[i][j] * k;					\
		}							\
	}

#define MTX_DIV_SCL							\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = a[i][j] / k;					\
		}							\
	}

#define MTX_ROW_SWP			\
	for (int j = 0; j < n; j++) {	\
		tmp = a[k][j];		\
		a[k][j] = a[l][j];	\
		a[l][j] = tmp;		\
	}

#define MTX_ROW_ADD			\
	for (int j = 0; j < n; j++) {	\
		a[l][j] += a[k][j];	\
	}

#define MTX_ROW_ADM			\
	for (int j = 0; j < n; j++) {	\
		a[l][j] += (x*a[k][j]);	\
	}

#define MTX_ROW_SUB			\
	for (int j = 0; j < n; j++) {	\
		a[l][j] -= a[k][j];	\
	}

#define MTX_ROW_ADD_SCL			\
	for (int j = 0; j < n; j++) {	\
		a[l][j] += k;		\
	}

#define MTX_ROW_SUB_SCL			\
	for (int j = 0; j < n; j++) {	\
		a[l][j] -= k;		\
	}

#define MTX_ROW_MUL_SCL			\
	for (int j = 0; j < n; j++) {	\
		a[l][j] *= k;		\
	}

#define MTX_ROW_DIV_SCL			\
	for (int j = 0; j < n; j++) {	\
		a[l][j] /= k;		\
	}

#define MTX_COL_SWP			\
	for (int i = 0; i < m; i++) { 	\
		tmp = a[i][k];		\
		a[i][k] = a[i][l];	\
		a[i][l] = tmp;		\
	}

#define MTX_COL_ADD			\
	for (int i = 0; i < m; i++) {	\
		a[i][l] += a[i][k];	\
	}

#define MTX_COL_ADM			\
	for (int i = 0; i < m; i++) {	\
		a[i][l] += (x*a[i][k]);	\
	}

#define MTX_COL_SUB			\
	for (int i = 0; i < m; i++) {	\
		a[i][l] -= a[i][k];	\
	}

#define MTX_COL_ADD_SCL			\
	for (int i = 0; i < m; i++) {	\
		a[i][l] += k;		\
	}

#define MTX_COL_SUB_SCL			\
	for (int i = 0; i < m; i++) {	\
		a[i][l] -= k;		\
	}

#define MTX_COL_MUL_SCL			\
	for (int i = 0; i < m; i++) {	\
		a[i][l] *= k;		\
	}

#define MTX_COL_DIV_SCL			\
	for (int i = 0; i < m; i++) {	\
		a[i][l] /= k;		\
	}

void mtxi_zer(const int m, const int n, int *C)
{
	int (*c)[n] = (int (*)[n])C;
	MTX_ZER;
}
void mtxf_zer(const int m, const int n, float *C)
{
	float (*c)[n] = (float (*)[n])C;
	MTX_ZER;
}
void mtxlf_zer(const int m, const int n, double *C)
{
	double (*c)[n] = (double (*)[n])C;
}
void mtxLf_zer(const int m, const int n, long double *C)
{
	long double (*c)[n] = (long double (*)[n])C;
	MTX_ZER;
}

void mtxi_eye(const int m, const int n, int *C)
{
	int (*c)[n] = (int (*)[n])C;
	MTX_EYE;
}
void mtxf_eye(const int m, const int n, float *C)
{
	float (*c)[n] = (float (*)[n])C;
	MTX_EYE;
}
void mtxlf_eye(const int m, const int n, double *C)
{
	double (*c)[n] = (double (*)[n])C;
	MTX_EYE;
}
void mtxLf_eye(const int m, const int n, long double *C)
{
	long double (*c)[n] = (long double (*)[n])C;
	MTX_EYE;
}

void mtxi_cpy(const int * const A, const int m, const int n, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*c)[n] = (int (*)[n])C;
	MTX_CPY;
}
void mtxf_cpy(const float * A, const int m, const int n, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*c)[n] = (float (*)[n])C;
	MTX_CPY;
}
void mtxlf_cpy(const double * A, const int m, const int n, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*c)[n] = (double (*)[n])C;
	MTX_CPY;
}
void mtxLf_cpy(const long double * A, const int m, const int n, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_CPY;
}

void mtxi_trn(const int * const A, const int m, const int n, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*c)[n] = (int (*)[n])C;
	MTX_TRN;
}
void mtxf_trn(const float * A, const int m, const int n, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*c)[n] = (float (*)[n])C;
	MTX_TRN;
}
void mtxlf_trn(const double * A, const int m, const int n, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*c)[n] = (double (*)[n])C;
	MTX_TRN;
}
void mtxLf_trn(const long double * A, const int m, const int n, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_TRN;
}
void mtxi_add(const int * const A, const int * const B, const int m, const int n, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*b)[n] = (int (*)[n])B;
	int (*c)[n] = (int (*)[n])C;
	MTX_ADD;
}
void mtxf_add(const float * A, const float * B, const int m, const int n, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*b)[n] = (float (*)[n])B;
	float (*c)[n] = (float (*)[n])C;
	MTX_ADD;
}
void mtxlf_add(const double * A, const double * B, const int m, const int n, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*b)[n] = (double (*)[n])B;
	double (*c)[n] = (double (*)[n])C;
	MTX_ADD;
}
void mtxLf_add(const long double * A, const long double * B, const int m, const int n, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*b)[n] = (long double (*)[n])B;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_ADD;
}
void mtxi_sub(const int * const A, const int * const B, const int m, const int n, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*b)[n] = (int (*)[n])B;
	int (*c)[n] = (int (*)[n])C;
	MTX_SUB;
}
void mtxf_sub(const float * A, const float * B, const int m, const int n, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*b)[n] = (float (*)[n])B;
	float (*c)[n] = (float (*)[n])C;
	MTX_SUB;
}
void mtxlf_sub(const double * A, const double * B, const int m, const int n, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*b)[n] = (double (*)[n])B;
	double (*c)[n] = (double (*)[n])C;
	MTX_SUB;
}
void mtxLf_sub(const long double * A, const long double * B, const int m, const int n, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*b)[n] = (long double (*)[n])B;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_SUB;
}
void mtxi_mul(const int * const A, const int * const B, const int m, const int p, const int n, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*b)[p] = (int (*)[n])B;
	int (*c)[p] = (int (*)[n])C;
	MTX_MUL;
}
void mtxf_mul(const float * A, const float * B, const int m, const int p, const int n, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*b)[p] = (float (*)[n])B;
	float (*c)[p] = (float (*)[n])C;
	MTX_MUL;
}
void mtxlf_mul(const double * A, const double * B, const int m, const int p, const int n, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*b)[p] = (double (*)[n])B;
	double (*c)[p] = (double (*)[n])C;
	MTX_MUL;
}
void mtxLf_mul(const long double * A, const long double * B, const int m, const int p, const int n, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*b)[p] = (long double (*)[n])B;
	long double (*c)[p] = (long double (*)[n])C;
	MTX_MUL;
}

/* C(mxn) = A(mxn) + k*J(mxn)	Multiply each element of A(mxn) by constant k */
void  mtxi_add_scl(const int * const A,  const int m, const int n, int k, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*c)[n] = (int (*)[n])C;
	MTX_ADD_SCL;
}
void  mtxf_add_scl(const float * const A, const int m, const int n, float k, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*c)[n] = (float (*)[n])C;
	MTX_ADD_SCL;
}
void mtxlf_add_scl(const double * const A, const int m, const int n, double k, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*c)[n] = (double (*)[n])C;
	MTX_ADD_SCL;
}
void mtxLf_add_scl(const long double * const A, const int m, const int n, long double k, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_ADD_SCL;
}

/* C(mxn) = A(mxn) - k*J(mxn)	Subtract constant k from each element of A(mxn) */
void  mtxi_sub_scl(const int * const A,  const int m, const int n, int k, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*c)[n] = (int (*)[n])C;
	MTX_SUB_SCL;
}
void  mtxf_sub_scl(const float * const A, const int m, const int n, float k, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*c)[n] = (float (*)[n])C;
	MTX_SUB_SCL;
}
void mtxlf_sub_scl(const double * const A, const int m, const int n, double k, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*c)[n] = (double (*)[n])C;
	MTX_SUB_SCL;
}
void mtxLf_sub_scl(const long double * const A, const int m, const int n, long double k, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_SUB_SCL;
}

/* C(mxn) = A(mxp) * k*J(mxn)	Multiply each element of A(mxn) by constant k */
void  mtxi_mul_scl(const int * const A,  const int m, const int n, int k, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*c)[n] = (int (*)[n])C;
	MTX_MUL_SCL;
}
void  mtxf_mul_scl(const float * const A, const int m, const int n, float k, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*c)[n] = (float (*)[n])C;
	MTX_MUL_SCL;
}
void mtxlf_mul_scl(const double * const A, const int m, const int n, double k, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*c)[n] = (double (*)[n])C;
	MTX_MUL_SCL;
}
void mtxLf_mul_scl(const long double * const A, const int m, const int n, long double k, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_MUL_SCL;
}

/* C(mxn) = A(mxp) / k*J(mxn)	Divide each element of A(mxn) by constant k */
void  mtxi_div_scl(const int * const A,  const int m, const int n, int k, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*c)[n] = (int (*)[n])C;
	MTX_DIV_SCL;
}
void  mtxf_div_scl(const float * const A, const int m, const int n, float k, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*c)[n] = (float (*)[n])C;
	MTX_DIV_SCL;
}
void mtxlf_div_scl(const double * const A, const int m, const int n, double k, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*c)[n] = (double (*)[n])C;
	MTX_DIV_SCL;
}
void mtxLf_div_scl(const long double * const A, const int m, const int n, long double k, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_DIV_SCL;
}

/* ROW OPERATIONS */
void mtxi_row_swp(int * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	int (*a)[n] = (int (*)[n])A;
	int tmp;
	MTX_ROW_SWP;
}
void mtxf_row_swp(float * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	float (*a)[n] = (float (*)[n])A;
	float tmp;
	MTX_ROW_SWP;
}
void mtxlf_row_swp(double * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	double (*a)[n] = (double (*)[n])A;
	double tmp;
	MTX_ROW_SWP;
}
void mtxLf_row_swp(long double * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	long double tmp;
	MTX_ROW_SWP;
}

void  mtxi_row_add(int * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	int (*a)[n] = (int (*)[n])A;
	MTX_ROW_ADD;
}
void  mtxf_row_add(float * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	float (*a)[n] = (float (*)[n])A;
	MTX_ROW_ADD;
}
void mtxlf_row_add(double * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	double (*a)[n] = (double (*)[n])A;
	MTX_ROW_ADD;
}
void mtxLf_row_add(long double * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	MTX_ROW_ADD;
}

void  mtxi_row_adm(int * A, const int n, int x, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	int (*a)[n] = (int (*)[n])A;
	MTX_ROW_ADM;
}
void  mtxf_row_adm(float * A, const int n, float x, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	float (*a)[n] = (float (*)[n])A;
	MTX_ROW_ADM;
}
void mtxlf_row_adm(double * A, const int n, double x, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	double (*a)[n] = (double (*)[n])A;
	MTX_ROW_ADM;
}
void mtxLf_row_adm(long double * A, const int n, long double x, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	MTX_ROW_ADM;
}

/*  L(n) += k	add scalar 'k' to row 'l' on matrix A(_xn) */
void  mtxi_row_add_scl(int * A, const int n, const int k, int l)
{
	l--; /* l index starts at 1 */
	int (*a)[n] = (int (*)[n])A;
	MTX_ROW_ADD_SCL;
}

void  mtxi_row_sub(int * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	int (*a)[n] = (int (*)[n])A;
	MTX_ROW_SUB;
}
void  mtxf_row_sub(float * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	float (*a)[n] = (float (*)[n])A;
	MTX_ROW_SUB;
}
void mtxlf_row_sub(double * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	double (*a)[n] = (double (*)[n])A;
	MTX_ROW_SUB;
}
void mtxLf_row_sub(long double * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	MTX_ROW_SUB;
}

void  mtxf_row_add_scl(float * A, const int n, const float k, int l)
{
	l--; /* l index starts at 1 */
	float (*a)[n] = (float (*)[n])A;
	MTX_ROW_ADD_SCL;
}
void mtxlf_row_add_scl(double * A, const int n, const double k, int l)
{
	l--; /* l index starts at 1 */
	double (*a)[n] = (double (*)[n])A;
	MTX_ROW_ADD_SCL;
}
void mtxLf_row_add_scl(long double * A, const int n, const long double k, int l)
{
	l--; /* l index starts at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	MTX_ROW_ADD_SCL;
}

/*  L(n) -= k	subtract scalar 'k' from row 'l' on matrix A(_xn) */
void  mtxi_row_sub_scl(int * A, const int n, const int k, int l)
{
	l--; /* l index starts at 1 */
	int (*a)[n] = (int (*)[n])A;
	MTX_ROW_SUB_SCL;
}
void  mtxf_row_sub_scl(float * A, const int n, const float k, int l)
{
	l--; /* l index starts at 1 */
	float (*a)[n] = (float (*)[n])A;
	MTX_ROW_SUB_SCL;
}
void mtxlf_row_sub_scl(double * A, const int n, const double k, int l)
{
	l--; /* l index starts at 1 */
	double (*a)[n] = (double (*)[n])A;
	MTX_ROW_SUB_SCL;
}
void mtxLf_row_sub_scl(long double * A, const int n, const long double k, int l)
{
	l--; /* l index starts at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	MTX_ROW_SUB_SCL;
}

/*  L(n) *= k	multiply row 'l' by scalar 'k' on matrix A(_xn) */
void  mtxi_row_mul_scl(int * A, const int n, const int k, int l)
{
	l--; /* l index starts at 1 */
	int (*a)[n] = (int (*)[n])A;
	MTX_ROW_MUL_SCL;
}
void  mtxf_row_mul_scl(float * A, const int n, const float k, int l)
{
	l--; /* l index starts at 1 */
	float (*a)[n] = (float (*)[n])A;
	MTX_ROW_MUL_SCL;
}
void mtxlf_row_mul_scl(double * A, const int n, const double k, int l)
{
	l--; /* l index starts at 1 */
	double (*a)[n] = (double (*)[n])A;
	MTX_ROW_MUL_SCL;
}
void mtxLf_row_mul_scl(long double * A, const int n, const long double k, int l)
{
	l--; /* l index starts at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	MTX_ROW_MUL_SCL;
}

/*  L(n) \= k	divide row 'l' by scalar 'k' on matrix A(_xn) */
void  mtxi_row_div_scl(int * A, const int n, const int k, int l)
{
	l--; /* l index starts at 1 */
	int (*a)[n] = (int (*)[n])A;
	MTX_ROW_DIV_SCL;
}
void  mtxf_row_div_scl(float * A, const int n, const float k, int l)
{
	l--; /* l index starts at 1 */
	float (*a)[n] = (float (*)[n])A;
	MTX_ROW_DIV_SCL;
}
void mtxlf_row_div_scl(double * A, const int n, const double k, int l)
{
	l--; /* l index starts at 1 */
	double (*a)[n] = (double (*)[n])A;
	MTX_ROW_DIV_SCL;
}
void mtxLf_row_div_scl(long double * A, const int n, const long double k, int l)
{
	l--; /* l index starts at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	MTX_ROW_DIV_SCL;
}


void mtxi_col_swp(int * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	int (*a)[m] = (int (*)[m])A;
	int tmp;
	MTX_COL_SWP;
}
void mtxf_col_swp(float * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	float (*a)[m] = (float (*)[m])A;
	float tmp;
	MTX_COL_SWP;
}
void mtxlf_col_swp(double * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	double (*a)[m] = (double (*)[m])A;
	double tmp;
	MTX_COL_SWP;
}
void mtxLf_col_swp(long double * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	long double tmp;
	MTX_COL_SWP;
}

/*  L(m) += K(m)		add column 'k' to column 'l' on matrix A(mx_) */
void  mtxi_col_add(int * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	int (*a)[m] = (int (*)[m])A;
	MTX_COL_ADD;
}
void  mtxf_col_add(float * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	float (*a)[m] = (float (*)[m])A;
	MTX_COL_ADD;
}
void mtxlf_col_add(double * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	double (*a)[m] = (double (*)[m])A;
	MTX_COL_ADD;
}
void mtxLf_col_add(long double * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	MTX_COL_ADD;
}

void  mtxi_col_adm(int * A, const int m, int x, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	int (*a)[m] = (int (*)[m])A;
	MTX_COL_ADM;
}
void  mtxf_col_adm(float * A, const int m, float x, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	float (*a)[m] = (float (*)[m])A;
	MTX_COL_ADM;
}
void mtxlf_col_adm(double * A, const int m, double x, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	double (*a)[m] = (double (*)[m])A;
	MTX_COL_ADM;
}
void mtxLf_col_adm(long double * A, const int m, long double x, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	MTX_COL_ADM;
}

/*  L(m) -= K(m)		subtract column 'k' from column 'l' on matrix A(mx_) */
void  mtxi_col_sub(int * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	int (*a)[m] = (int (*)[m])A;
	MTX_COL_SUB;
}
void  mtxf_col_sub(float * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	float (*a)[m] = (float (*)[m])A;
	MTX_COL_SUB;
}
void mtxlf_col_sub(double * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	double (*a)[m] = (double (*)[m])A;
	MTX_COL_SUB;
}
void mtxLf_col_sub(long double * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	MTX_COL_SUB;
}

/* L(m) += k	add scalar 'k' to col 'l' on matrix A(mx_) */
void  mtxi_col_add_scl(int * A, const int m, const int k, int l)
{
	l--; /* l index starts at 1 */
	int (*a)[m] = (int (*)[m])A;
	MTX_COL_ADD_SCL;
}
void  mtxf_col_add_scl(float * A, const int m, const float k, int l)
{
	l--; /* l index starts at 1 */
	float (*a)[m] = (float (*)[m])A;
	MTX_COL_ADD_SCL;
}
void mtxlf_col_add_scl(double * A, const int m, const double k, int l)
{
	l--; /* l index starts at 1 */
	double (*a)[m] = (double (*)[m])A;
	MTX_COL_ADD_SCL;
}
void mtxLf_col_add_scl(long double * A, const int m, const long double k, int l)
{
	l--; /* l index starts at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	MTX_COL_ADD_SCL;
}

/* L(m) -= k	subtract scalar 'k' from col 'l' on matrix A(mx_) */
void  mtxi_col_sub_scl(int * A, const int m, int k, int l)
{
	l--; /* l index starts at 1 */
	int (*a)[m] = (int (*)[m])A;
	MTX_COL_SUB_SCL;
}
void  mtxf_col_sub_scl(float * A, const int m, const float k, int l)
{
	float (*a)[m] = (float (*)[m])A;
	MTX_COL_SUB_SCL;
}
void mtxlf_col_sub_scl(double * A, const int m, const double k, int l)
{
	l--; /* l index starts at 1 */
	double (*a)[m] = (double (*)[m])A;
	MTX_COL_SUB_SCL;
}
void mtxLf_col_sub_scl(long double * A, const int m, const long double k, int l)
{
	l--; /* l index starts at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	MTX_COL_SUB_SCL;
}

/* L(m) *= k	multiply col 'l' by scalar 'k' on matrix A(mx_) */
void  mtxi_col_mul_scl(int * A, const int m, const int k, int l)
{
	l--; /* l index starts at 1 */
	int (*a)[m] = (int (*)[m])A;
	MTX_COL_MUL_SCL;
}
void  mtxf_col_mul_scl(float * A, const int m, const float k, int l)
{
	l--; /* l index starts at 1 */
	float (*a)[m] = (float (*)[m])A;
	MTX_COL_MUL_SCL;
}
void mtxlf_col_mul_scl(double * A, const int m, const double k, int l)
{
	l--; /* l index starts at 1 */
	double (*a)[m] = (double (*)[m])A;
	MTX_COL_MUL_SCL;
}
void mtxLf_col_mul_scl(long double * A, const int m, const long double k, int l)
{
	l--; /* l index starts at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	MTX_COL_MUL_SCL;
}

/* L(m) \= k	divide col 'l' by scalar 'k' on matrix A(mx_) */
void  mtxi_col_div_scl(int * A, const int m, const int k, int l)
{
	l--; /* l index starts at 1 */
	int (*a)[m] = (int (*)[m])A;
	MTX_COL_DIV_SCL;
}
void  mtxf_col_div_scl(float * A, const int m, const float k, int l)
{
	l--; /* l index starts at 1 */
	float (*a)[m] = (float (*)[m])A;
	MTX_COL_DIV_SCL;
}
void mtxlf_col_div_scl(double * A, const int m, const double k, int l)
{
	l--; /* l index starts at 1 */
	double (*a)[m] = (double (*)[m])A;
	MTX_COL_DIV_SCL;
}
void mtxLf_col_div_scl(long double * A, const int m, const long double k, int l)
{
	l--; /* l index starts at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	MTX_COL_DIV_SCL;
}

/*
 * Inverts a matrix using the Gauss Jordan method.
 * returns 0 on success, 1 on failure.
 */
#include <math.h>
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


void mtxi_pri(const int * const A, const int m, const int n)
{
	int (*a)[n] = (int (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			printf("%d ", a[i][j]);
		}
		printf("\n");
	}
}
void mtxf_pri(const float * const A, const int m, const int n)
{
	float (*a)[n] = (float (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			printf("%f ", a[i][j]);
		}
		printf("\n");
	}
}
void mtxlf_pri(double *A, const int m, const int n)
{
	double (*a)[n] = (double (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			printf("%lf ", a[i][j]);
		}
		printf("\n");
	}
}
void mtxLf_pri(long double *A, const int m, const int n)
{
	long double (*a)[n] = (long double (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			printf("%Lf ", a[i][j]);
		}
		printf("\n");
	}
}
void mtxi_pri_row(const int * const A, const int n, int k)
{
	k--; /* indexes start at 1 */
	int (*a)[n] = (int (*)[n])A;
	for (int j = 0; j < n; j++) {
		printf("%d ", a[k][j]);
	}
	printf("\n");
}
void mtxf_pri_row(const float * const A, const int n, int k)
{
	k--; /* indexes start at 1 */
	float (*a)[n] = (float (*)[n])A;
	for (int j = 0; j < n; j++) {
		printf("%f ", a[k][j]);
	}
	printf("\n");
}
void mtxlf_pri_row(const double * const A, const int n, int k)
{
	k--; /* indexes start at 1 */
	double (*a)[n] = (double (*)[n])A;
	for (int j = 0; j < n; j++) {
		printf("%lf ", a[k][j]);
	}
	printf("\n");
}
void mtxLf_pri_row(const long double * const A, const int n, int k)
{
	k--; /* indexes start at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	for (int j = 0; j < n; j++) {
		printf("%Lf ", a[k][j]);
	}
	printf("\n");
}
void mtxi_pri_col(const int * const A, const int m, int k)
{
	k--; /* indexes start at 1 */
	int (*a)[m] = (int (*)[m])A;
	for (int i = 0; i < m; i++) {
		printf("%d\n", a[i][k]);
	}
}
void mtxf_pri_col(const float * const A, const int m, int k)
{
	k--; /* indexes start at 1 */
	float (*a)[m] = (float (*)[m])A;
	for (int i = 0; i < m; i++) {
		printf("%f\n", a[i][k]);
	}
}
void mtxlf_pri_col(const double * const A, const int m, int k)
{
	k--; /* indexes start at 1 */
	double (*a)[m] = (double (*)[m])A;
	for (int i = 0; i < m; i++) {
		printf("%lf\n", a[i][k]);
	}
}
void mtxLf_pri_col(const long double * const A, const int m, int k)
{
	k--; /* indexes start at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	for (int i = 0; i < m; i++) {
		printf("%Lf\n", a[i][k]);
	}
}
void mtxi_spr(const int * const A, const int m, const int n, char *s)
{
	int (*a)[n] = (int (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			sprintf(s, "%d ", a[i][j]);
		}
		sprintf(s, "\n");
	}
}
void mtxf_spr(const float * const A, const int m, const int n, char *s)
{
	float (*a)[n] = (float (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			sprintf(s, "%f ", a[i][j]);
		}
		sprintf(s, "\n");
	}
}
void mtxlf_spr(double *A, const int m, const int n, char *s)
{
	double (*a)[n] = (double (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			sprintf(s, "%lf ", a[i][j]);
		}
		sprintf(s, "\n");
	}
}
void mtxLf_spr(long double *A, const int m, const int n, char *s)
{
	long double (*a)[n] = (long double (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			sprintf(s, "%Lf ", a[i][j]);
		}
		sprintf(s, "\n");
	}
}
void mtxi_spr_row(const int * const A, const int n, int k, char *s)
{
	k--; /* indexes start at 1 */
	int (*a)[n] = (int (*)[n])A;
	for (int j = 0; j < n; j++) {
		sprintf(s, "%d ", a[k][j]);
	}
	sprintf(s, "\n");
}
void mtxf_spr_row(const float * const A, const int n, int k, char *s)
{
	k--; /* indexes start at 1 */
	float (*a)[n] = (float (*)[n])A;
	for (int j = 0; j < n; j++) {
		sprintf(s, "%f ", a[k][j]);
	}
	sprintf(s, "\n");
}
void mtxlf_spr_row(const double * const A, const int n, int k, char *s)
{
	k--; /* indexes start at 1 */
	double (*a)[n] = (double (*)[n])A;
	for (int j = 0; j < n; j++) {
		sprintf(s, "%lf ", a[k][j]);
	}
	sprintf(s, "\n");
}
void mtxLf_spr_row(const long double * const A, const int n, int k, char *s)
{
	k--; /* indexes start at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	for (int j = 0; j < n; j++) {
		sprintf(s, "%Lf ", a[k][j]);
	}
	sprintf(s, "\n");
}
void mtxi_spr_col(const int * const A, const int m, int k, char *s)
{
	k--; /* indexes start at 1 */
	int (*a)[m] = (int (*)[m])A;
	for (int i = 0; i < m; i++) {
		sprintf(s, "%d\n", a[i][k]);
	}
}
void mtxf_spr_col(const float * const A, const int m, int k, char *s)
{
	k--; /* indexes start at 1 */
	float (*a)[m] = (float (*)[m])A;
	for (int i = 0; i < m; i++) {
		sprintf(s, "%f\n", a[i][k]);
	}
}
void mtxlf_spr_col(const double * const A, const int m, int k, char *s)
{
	k--; /* indexes start at 1 */
	double (*a)[m] = (double (*)[m])A;
	for (int i = 0; i < m; i++) {
		sprintf(s, "%lf\n", a[i][k]);
	}
}
void mtxLf_spr_col(const long double * const A, const int m, int k, char *s)
{
	k--; /* indexes start at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	for (int i = 0; i < m; i++) {
		sprintf(s, "%Lf\n", a[i][k]);
	}
}

void mtxi_fpr(const int * const A, const int m, const int n, FILE *f)
{
	int (*a)[n] = (int (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			fprintf(f, "%d ", a[i][j]);
		}
		fprintf(f, "\n");
	}
}
void mtxf_fpr(const float * const A, const int m, const int n, FILE *f)
{
	float (*a)[n] = (float (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			fprintf(f, "%f ", a[i][j]);
		}
		fprintf(f, "\n");
	}
}
void mtxlf_fpr(double *A, const int m, const int n, FILE *f)
{
	double (*a)[n] = (double (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			fprintf(f, "%lf ", a[i][j]);
		}
		fprintf(f, "\n");
	}
}
void mtxLf_fpr(long double *A, const int m, const int n, FILE *f)
{
	long double (*a)[n] = (long double (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			fprintf(f, "%Lf ", a[i][j]);
		}
		fprintf(f, "\n");
	}
}
void mtxi_fpr_row(const int * const A, const int n, int k, FILE *f)
{
	k--; /* indexes start at 1 */
	int (*a)[n] = (int (*)[n])A;
	for (int j = 0; j < n; j++) {
		fprintf(f, "%d ", a[k][j]);
	}
	fprintf(f, "\n");
}
void mtxf_fpr_row(const float * const A, const int n, int k, FILE *f)
{
	k--; /* indexes start at 1 */
	float (*a)[n] = (float (*)[n])A;
	for (int j = 0; j < n; j++) {
		fprintf(f, "%f ", a[k][j]);
	}
	fprintf(f, "\n");
}
void mtxlf_fpr_row(const double * const A, const int n, int k, FILE *f)
{
	k--; /* indexes start at 1 */
	double (*a)[n] = (double (*)[n])A;
	for (int j = 0; j < n; j++) {
		fprintf(f, "%lf ", a[k][j]);
	}
	fprintf(f, "\n");
}
void mtxLf_fpr_row(const long double * const A, const int n, int k, FILE *f)
{
	k--; /* indexes start at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	for (int j = 0; j < n; j++) {
		fprintf(f, "%Lf ", a[k][j]);
	}
	fprintf(f, "\n");
}
void mtxi_fpr_col(const int * const A, const int m, int k, FILE *f)
{
	k--; /* indexes start at 1 */
	int (*a)[m] = (int (*)[m])A;
	for (int i = 0; i < m; i++) {
		fprintf(f, "%d\n", a[i][k]);
	}
}
void mtxf_fpr_col(const float * const A, const int m, int k, FILE *f)
{
	k--; /* indexes start at 1 */
	float (*a)[m] = (float (*)[m])A;
	for (int i = 0; i < m; i++) {
		fprintf(f, "%f\n", a[i][k]);
	}
}
void mtxlf_fpr_col(const double * const A, const int m, int k, FILE *f)
{
	k--; /* indexes start at 1 */
	double (*a)[m] = (double (*)[m])A;
	for (int i = 0; i < m; i++) {
		fprintf(f, "%lf\n", a[i][k]);
	}
}
void mtxLf_fpr_col(const long double * const A, const int m, int k, FILE *f)
{
	k--; /* indexes start at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	for (int i = 0; i < m; i++) {
		fprintf(f, "%Lf\n", a[i][k]);
	}
}

