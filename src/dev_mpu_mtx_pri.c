#include "dev_mpu_mtx.h"
#include <stdio.h>
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
