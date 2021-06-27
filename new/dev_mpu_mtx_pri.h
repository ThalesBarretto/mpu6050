
#ifndef MPU_MTX_PRI_H_
#define MPU_MTX_PRI_H_
#include <stdio.h>

#if __STDC_VERSION__ >= 201112L

#define mtx_pri(A, m, n) \
	_Generic((A),\
	int*			:  mtxi_pri,\
	float*			:  mtxf_pri,\
	double*			: mtxlf_pri,\
	long double*		: mtxLf_pri\
	)((A),(m),(n))

#define mtx_pri_row(A, m, n) \
	_Generic((A),\
	int*			:  mtxi_pri_row,\
	float*			:  mtxf_pri_row,\
	double*			: mtxlf_pri_row,\
	long double*		: mtxLf_pri_row\
	)((A),(m),(n))

#define mtx_pri_col(A, m, n) \
	_Generic((A),\
	int*			:  mtxi_pri_col,\
	float*			:  mtxf_pri_col,\
	double*			: mtxlf_pri_col,\
	long double*		: mtxLf_pri_col\
	)((A),(m),(n))


#define mtx_spr(A, m, n, C) \
	_Generic((A),\
	int*			:  mtxi_spr,\
	float*			:  mtxf_spr,\
	double*			: mtxlf_spr,\
	long double*		: mtxLf_spr\
	)((A),(m),(n),(C))

#define mtx_spr_row(A, m, n, C) \
	_Generic((A),\
	int*			:  mtxi_spr_row,\
	float*			:  mtxf_spr_row,\
	double*			: mtxlf_spr_row,\
	long double*		: mtxLf_spr_row\
	)((A),(m),(n),(C))

#define mtx_spr_col(A, m, n, C) \
	_Generic((A),\
	int*			:  mtxi_spr_col,\
	float*			:  mtxf_spr_col,\
	double*			: mtxlf_spr_col,\
	long double*		: mtxLf_spr_col\
	)((A),(m),(n),(C))

#define mtx_fpr(A, m, n, C) \
	_Generic((A),\
	int*			:  mtxi_fpr,\
	float*			:  mtxf_fpr,\
	double*			: mtxlf_fpr,\
	long double*		: mtxLf_fpr\
	)((A),(m),(n),(C))

#define mtx_fpr_row(A, m, n, C) \
	_Generic((A),\
	int*			:  mtxi_fpr_row,\
	float*			:  mtxf_fpr_row,\
	double*			: mtxlf_fpr_row,\
	long double*		: mtxLf_fpr_row\
	)((A),(m),(n),(C))

#define mtx_fpr_col(A, m, n, C) \
	_Generic((A),\
	int*			:  mtxi_fpr_col,\
	float*			:  mtxf_fpr_col,\
	double*			: mtxlf_fpr_col,\
	long double*		: mtxLf_fpr_col\
	)((A),(m),(n),(C))

#endif /* __STDC_VERSION__ */

void  mtxi_pri(const int * const A, const int m, const int n);
void  mtxf_pri(const float * const A, const int m, const int n);
void mtxlf_pri(double *A, const int m, const int n);
void mtxLf_pri(long double *A, const int m, const int n);
void  mtxi_pri_row(const int * const A, const int n, int k);
void  mtxf_pri_row(const float * const A, const int n, int k);
void mtxlf_pri_row(const double * const A, const int n, int k);
void mtxLf_pri_row(const long double * const A, const int n, int k);
void  mtxi_pri_col(const int * const A, const int m, int k);
void  mtxf_pri_col(const float * const A, const int m, int k);
void mtxlf_pri_col(const double * const A, const int m, int k);
void mtxLf_pri_col(const long double * const A, const int m, int k);

void  mtxi_spr(const int * const A, const int m, const int n, char *s);
void  mtxf_spr(const float * const A, const int m, const int n, char *s);
void mtxlf_spr(double *A, const int m, const int n, char *s);
void mtxLf_spr(long double *A, const int m, const int n, char *s);
void  mtxi_spr_row(const int * const A, const int n, int k, char *s);
void  mtxf_spr_row(const float * const A, const int n, int k, char *s);
void mtxlf_spr_row(const double * const A, const int n, int k, char *s);
void mtxLf_spr_row(const long double * const A, const int n, int k, char *s);
void  mtxi_spr_col(const int * const A, const int m, int k, char *s);
void  mtxf_spr_col(const float * const A, const int m, int k, char *s);
void mtxlf_spr_col(const double * const A, const int m, int k, char *s);
void mtxLf_spr_col(const long double * const A, const int m, int k, char *s);

void  mtxi_fpr(const int * const A, const int m, const int n, FILE *f);
void  mtxf_fpr(const float * const A, const int m, const int n, FILE *f);
void mtxlf_fpr(double *A, const int m, const int n, FILE *f);
void mtxLf_fpr(long double *A, const int m, const int n, FILE *f);
void  mtxi_fpr_row(const int * const A, const int n, int k, FILE *f);
void  mtxf_fpr_row(const float * const A, const int n, int k, FILE *f);
void mtxlf_fpr_row(const double * const A, const int n, int k, FILE *f);
void mtxLf_fpr_row(const long double * const A, const int n, int k, FILE *f);
void  mtxi_fpr_col(const int * const A, const int m, int k, FILE *f);
void  mtxf_fpr_col(const float * const A, const int m, int k, FILE *f);
void mtxlf_fpr_col(const double * const A, const int m, int k, FILE *f);
void mtxLf_fpr_col(const long double * const A, const int m, int k, FILE *f);

#endif /* MPU_MTX_PRI_H_ */
