#ifdef __cplusplus
	extern "C" {
#endif

/* C11 or later required for type-generic macros */
#if __STDC_VERSION__ >= 201112L

#ifndef MPU_MTX_H_
#define MPU_MTX_H_
#include <math.h>
#include <tgmath.h>
/*
 * MATRIX OPERATIONS
 * Conventions:
 * 	- Types: int, float, double, long double
 * 	- Types are chosen according to the destination argument
 * 	- Do not mix types of A, B and C (unsupported)
 * 	- Matrix dimensions are (nxn),(mxn),(mxp)(pxn)
 * 	- indexes in the loops are'i' for row, 'j' for column
 * 	- A, B operands are not modified, C is modified (C = A*B)
 * 	- operands and destination must not overlap
 * 	- A is modified in-place by "_row_" and "_col_" operations
 *
 * Matrix Inversion:
 * 	- A is the source (not modified)
 * 	- B is the buffer (overwritten)
 * 	- C is the destination (C = A_inverse)
 * 	- no checks performed (you must provide an invertible matrix)
 * 	- only fails when there is a zero on the pivot
 * 	- returns 0 on success, -1 on failure
 *
 * Rotation Matrices:
 * 	- given angles unit must chosen to be RADS or DEGS
 * 	- returned angles unit must chosen to be RADS or DEGS
 * 	- Rotation Matrices are (4x4)
 * 	- Euler angles are (3x1)[phi, theta, psi]
 * 	- Quaterions are   (4x1)[q0, q1, q2, q3]
 * 	- *A is always the source
 * 	- *C is always the destination
 * 	- no checks performed
 *
 * mtx_zer		Matrix Zero
 * mtx_eye		Matrix Identity
 * mtx_rox		4x4 X-axis rotation Matrix
 * mtx_roy		4x4 Y-axis rotation Matrix
 * mtx_roz		4x4 Z-axis rotation Matrix
 * mtx_cpy		Matrix Copy
 * mtx_trn		Matrix Transpose
 * mtx_det		Matrix Determinant
 * mtx_inv		Matrix Inversion
 * mtx_add		Matrix Addition
 * mtx_sub		Matrix Subtraction
 * mtx_mul		Matrix Multiplication
 * mtx_add_sca		Matrix Addition by a scalar
 * mtx_sub_sca		Matrix Subtraction by a scalar
 * mtx_mul_sca		Matrix Multiplication by a scalar
 * mtx_row_swp		Matrix Row Swap
 * mtx_row_add		Matrix Row Addition
 * mtx_row_adm		Matrix Row Multiple Addition
 * mtx_row_sub		Matrix Row Subtraction
 * mtx_row_add_scl	Matrix Row Addition by a scalar
 * mtx_row_sub_scl	Matrix Row Subtraction by a scalar
 * mtx_row_mul_scl	Matrix Row Multiplication by a scalar
 * mtx_row_div_scl	Matrix Row Division by a scalar
 * mtx_col_swp		Matrix Column Swap
 * mtx_col_add		Matrix Column Addition
 * mtx_col_adm		Matrix Column Multiple Addition
 * mtx_col_sub		Matrix Column Subtraction
 * mtx_col_add_scl	Matrix Column Addition by a scalar
 * mtx_col_sub_scl	Matrix Column Subtraction by a scalar
 * mtx_col_mul_scl	Matrix Column Multiplication by a scalar
 * mtx_col_div_scl	Matrix Column Division by a scalar
 *
 * vec_add		Vector Addition
 * vec_sub		Vector Subtraction
 * vec_dot		Vector Dot Product
 * vec_cro		Vector Cross Product
 * vec_add_sca		Vector Addition by a scalar
 * vec_sub_sca		Vector Subtraction by a scalar
 * vec_mul_sca		Vector Multiplication by a scalar
 */

#define mtx_zer(m, n, C)			\
	_Generic((C),				\
	int*		:  mtxi_zer,	 	\
	float*		:  mtxf_zer,	 	\
	double*		: mtxlf_zer,	 	\
	long double*	: mtxLf_zer	 	\
	)((m), (n), (C))

#define mtx_eye(m, n, C)			\
	_Generic((C),				\
	int*		:  mtxi_eye,	 	\
	float*		:  mtxf_eye,	 	\
	double*		: mtxlf_eye,	 	\
	long double*	: mtxLf_eye	 	\
	)((m), (n), (C))

#define mtx_cpy(A, m, n, C)			\
	_Generic((C),				\
	int*		:  mtxi_cpy,	 	\
	float*		:  mtxf_cpy,	 	\
	double*		: mtxlf_cpy,	 	\
	long double*	: mtxLf_cpy	 	\
	)((A), (m), (n), (C))

#define mtx_trn(A, m, n, C)			\
	_Generic((C),				\
	int*		:  mtxi_trn,	 	\
	float*		:  mtxf_trn,	 	\
	double*		: mtxlf_trn,	 	\
	long double*	: mtxLf_trn	 	\
	)((A), (m), (n), (C))

#define mtx_inv(A, B, n, C)			\
	_Generic((C),				\
	int*		:  mtxi_inv,	 	\
	float*		:  mtxf_inv,	 	\
	double*		: mtxlf_inv,	 	\
	long double*	: mtxLf_inv	 	\
	)((A), (B), (n), (C))

#define mtx_add(A, B, m, n, C)			\
	_Generic((C),				\
	int*		:  mtxi_add,		\
	float*		:  mtxf_add,		\
	double*		: mtxlf_add,		\
	long double*	: mtxLf_add		\
	)((A), (B), (m), (n), (C))

#define mtx_sub(A, B, m, n, C)			\
	_Generic((C),				\
	int*		:  mtxi_sub,		\
	float*		:  mtxf_sub,		\
	double*		: mtxlf_sub,		\
	long double*	: mtxLf_sub		\
	)((A), (B), (m), (n), (C))

#define mtx_mul(A, B, m, p, n, C)		\
	_Generic((C),				\
	int*		:  mtxi_mul,		\
	float*		:  mtxf_mul,		\
	double*		: mtxlf_mul,		\
	long double*	: mtxLf_mul		\
	)((A), (B), (m), (p), (n), (C))

#define mtx_add_scl(A, m, n, k, C)		\
	_Generic((C),				\
	int*		:  mtxi_add_scl,	\
	float*		:  mtxf_add_scl,	\
	double*		: mtxlf_add_scl,	\
	long double*	: mtxLf_add_scl		\
	)((A), (m), (n), (k), (C))

#define mtx_sub_scl(A, m, n, k, C)		\
	_Generic((C),				\
	int*		:  mtxi_sub_scl,	\
	float*		:  mtxf_sub_scl,	\
	double*		: mtxlf_sub_scl,	\
	long double*	: mtxLf_sub_scl		\
	)((A), (m), (n), (k), (C))

#define mtx_mul_scl(A, m, n, k, C)		\
	_Generic((C),				\
	int*		:  mtxi_mul_scl,	\
	float*		:  mtxf_mul_scl,	\
	double*		: mtxlf_mul_scl,	\
	long double*	: mtxLf_mul_scl		\
	)((A), (m), (n), (k), (C))

#define mtx_div_scl(A, m, n, k, C)		\
       	_Generic((C),				\
	int*		:  mtxi_div_scl,	\
	float*		:  mtxf_div_scl,	\
	double*		: mtxlf_div_scl,	\
	long double*	: mtxLf_div_scl		\
	)((A), (m), (n), (k), (C))

#define mtx_row_swp(A, n, k, l)			\
	_Generic((A),				\
	int*		:  mtxi_row_swp, 	\
	float*		:  mtxf_row_swp, 	\
	double*		: mtxlf_row_swp, 	\
	long double*	: mtxLf_row_swp	 	\
	)((A), (n), (k), (l))

#define mtx_row_add(A, n, k, l)			\
	_Generic((A),				\
	int*		:  mtxi_row_add, 	\
	float*		:  mtxf_row_add, 	\
	double*		: mtxlf_row_add, 	\
	long double*	: mtxLf_row_add	 	\
	)((A), (n), (k), (l))

#define mtx_row_adm(A, n, x, k, l)		\
	_Generic((A),				\
	int*		:  mtxi_row_adm, 	\
	float*		:  mtxf_row_adm, 	\
	double*		: mtxlf_row_adm, 	\
	long double*	: mtxLf_row_adm	 	\
	)((A), (n), (x), (k), (l))

#define mtx_row_sub(A, n, k, l)			\
	_Generic((A),				\
	int*		:  mtxi_row_sub, 	\
	float*		:  mtxf_row_sub, 	\
	double*		: mtxlf_row_sub, 	\
	long double*	: mtxLf_row_sub	 	\
	)((A), (n), (k), (l))

#define mtx_row_add_scl(A, n, k, l)		\
	_Generic((A),				\
	int*		:  mtxi_row_add_scl, 	\
	float*		:  mtxf_row_add_scl, 	\
	double*		: mtxlf_row_add_scl, 	\
	long double*	: mtxLf_row_add_scl 	\
	)((A), (n), (k), (l))

#define mtx_row_sub_scl(A, n, k, l)		\
	_Generic((A),				\
	int*		:  mtxi_row_sub_scl, 	\
	float*		:  mtxf_row_sub_scl, 	\
	double*		: mtxlf_row_sub_scl, 	\
	long double*	: mtxLf_row_sub_scl 	\
	)((A), (n), (k), (l))

#define mtx_row_mul_scl(A, n, k, l)		\
	_Generic((A),				\
	int*		:  mtxi_row_mul_scl, 	\
	float*		:  mtxf_row_mul_scl, 	\
	double*		: mtxlf_row_mul_scl, 	\
	long double*	: mtxLf_row_mul_scl 	\
	)((A), (n), (k), (l))

#define mtx_row_div_scl(A, n, k, l)		\
	_Generic((A),				\
	int*		:  mtxi_row_div_scl, 	\
	float*		:  mtxf_row_div_scl, 	\
	double*		: mtxlf_row_div_scl, 	\
	long double*	: mtxLf_row_div_scl 	\
	)((A), (n), (k), (l))

#define mtx_col_swp(A, m, k, l)			\
	_Generic((A),				\
	int*		:  mtxi_col_swp,	\
	float*		:  mtxf_col_swp,	\
	double*		: mtxlf_col_swp,	\
	long double*	: mtxLf_col_swp		\
	)((A), (m), (k), (l))

#define mtx_col_add(A, m, k, l)			\
	_Generic((A),				\
	int*		:  mtxi_col_add,	\
	float*		:  mtxf_col_add,	\
	double*		: mtxlf_col_add,	\
	long double*	: mtxLf_col_add		\
	)((A), (m), (k), (l))

#define mtx_col_adm(A, m, x, k, l)		\
	_Generic((A),				\
	int*		:  mtxi_col_adm,	\
	float*		:  mtxf_col_adm,	\
	double*		: mtxlf_col_adm,	\
	long double*	: mtxLf_col_adm		\
	)((A), (m), (x), (k), (l))

#define mtx_col_sub(A, m, k, l)			\
	_Generic((A),				\
	int*		:  mtxi_col_sub,	\
	float*		:  mtxf_col_sub,	\
	double*		: mtxlf_col_sub,	\
	long double*	: mtxLf_col_sub		\
	)((A), (m), (k), (l))

#define mtx_col_add_scl(A, m, k, l)		\
	_Generic((A),				\
	int*		:  mtxi_col_add_scl,	\
	float*		:  mtxf_col_add_scl,	\
	double*		: mtxlf_col_add_scl,	\
	long double*	: mtxLf_col_add_scl	\
	)((A), (m), (k), (l))

#define mtx_col_sub_scl(A, m, k, l)		\
	_Generic((A),				\
	int*		:  mtxi_col_sub_scl,	\
	float*		:  mtxf_col_sub_scl,	\
	double*		: mtxlf_col_sub_scl,	\
	long double*	: mtxLf_col_sub_scl	\
	)((A), (n), (k), (l))

#define mtx_col_mul_scl(A, m, k, l)		\
	_Generic((A),				\
	int*		:  mtxi_col_mul_scl,	\
	float*		:  mtxf_col_mul_scl,	\
	double*		: mtxlf_col_mul_scl,	\
	long double*	: mtxLf_col_mul_scl	\
	)((A), (n), (k), (l))

#define mtx_col_div_scl(A, m, k, l)		\
	_Generic((A),				\
	int*		:  mtxi_col_div_scl,	\
	float*		:  mtxf_col_div_scl,	\
	double*		: mtxlf_col_div_scl,	\
	long double*	: mtxLf_col_div_scl	\
	)((A), (n), (k), (l))


/* C(mxn) = 0 */
void  mtxi_zer(const int m, const int n, int *C);
void  mtxf_zer(const int m, const int n, float *C);
void mtxlf_zer(const int m, const int n, double *C);
void mtxLf_zer(const int m, const int n, long double *C);

/* C(mxn) = I */
void  mtxi_eye(const int m, const int n, int *C);
void  mtxf_eye(const int m, const int n, float *C);
void mtxlf_eye(const int m, const int n, double *C);
void mtxLf_eye(const int m, const int n, long double *C);


/* C(mxn) = A(mxn) */
void  mtxi_cpy(const int * const A, const int m, const int n, int *C);
void  mtxf_cpy(const float * A, const int m, const int n, float *C);
void mtxlf_cpy(const double * A, const int m, const int n, double *C);
void mtxLf_cpy(const long double * A, const int m, const int n, long double *C);

/* C(mxn) = A(mxn)_tranpose */
void  mtxi_trn(const int * const A, const int m, const int n, int *C);
void  mtxf_trn(const float * A, const int m, const int n, float *C);
void mtxlf_trn(const double * A, const int m, const int n, double *C);
void mtxLf_trn(const long double * A, const int m, const int n, long double *C);

/* C(mxn) = A(mxn)_inverse */
int  mtxi_inv(int *  A, int * B, const int n, int *C);
int  mtxf_inv(float * A, float * B, const int n, float *C);
int mtxlf_inv(double * A, double * B, const int n, double *C);
int mtxLf_inv(long double * A, long double * B, const int n, long double *C);

/* C(mxn) = A(mxn) + B(mxn) */
void  mtxi_add(const int * const A, const int * const B, const int m, const int n, int *C);
void  mtxf_add(const float * A, const float * B, const int m, const int n, float *C);
void mtxlf_add(const double * A, const double * B, const int m, const int n, double *C);
void mtxLf_add(const long double * A, const long double * B, const int m, const int n, long double *C);

/* C(mxn) = A(mxn) - B(mxn) */
void  mtxi_sub(const int * const A, const int * const B, const int m, const int n, int *C);
void  mtxf_sub(const float * A, const float * B, const int m, const int n, float *C);
void mtxlf_sub(const double * A, const double * B, const int m, const int n, double *C);
void mtxLf_sub(const long double * A, const long double * B, const int m, const int n, long double *C);

/* C(mxn) = A(mxp) * B(pxn) */
void  mtxi_mul(const int * const A, const int * const B, const int m, int p, const int n, int *C);
void  mtxf_mul(const float * A, const float * B, const int m, int p, const int n, float *C);
void mtxlf_mul(const double * A, const double * B, const int m, int p, const int n, double *C);
void mtxLf_mul(const long double * A, const long double * B, const int m, int p, const int n, long double *C);

/* C(mxn) = A(mxn) + k*J(mxn)	Multiply each element of A(mxn) by constant k */
void  mtxi_add_scl(const int * const A,  const int m, const int n, const int k, int *C);
void  mtxf_add_scl(const float * const A, const int m, const int n, const float k, float *C);
void mtxlf_add_scl(const double * const A, const int m, const int n, const double k, double *C);
void mtxLf_add_scl(const long double * const A, const int m, const int n, const long double k, long double *C);

/* C(mxn) = A(mxn) - k*J(mxn)	Subtract constant k from each element of A(mxn) */
void  mtxi_sub_scl(const int * const A,  const int m, const int n, const int k, int *C);
void  mtxf_sub_scl(const float * const A, const int m, const int n, const float k, float *C);
void mtxlf_sub_scl(const double * const A, const int m, const int n, const double k, double *C);
void mtxLf_sub_scl(const long double * const A, const int m, const int n, const long double k, long double *C);

/* C(mxn) = A(mxn) * k*J(mxn)	Multiply each element of A(mxn) by constant k */
void  mtxi_mul_scl(const int * const A,  const int m, const int n, const int k, int *C);
void  mtxf_mul_scl(const float * const A, const int m, const int n, const float k, float *C);
void mtxlf_mul_scl(const double * const A, const int m, const int n, const double k, double *C);
void mtxLf_mul_scl(const long double * const A, const int m, const int n, const long double k, long double *C);

/* C(mxn) = A(mxn) / k*J(mxn)	Divide each element of A(mxn) by constant k */
void  mtxi_div_scl(const int * const A,  const int m, const int n, const int k, int *C);
void  mtxf_div_scl(const float * const A, const int m, const int n, const float k, float *C);
void mtxlf_div_scl(const double * const A, const int m, const int n, const double k, double *C);
void mtxLf_div_scl(const long double * const A, const int m, const int n, const long double k, long double *C);

/* A(l,[0,n]) <-> A(k,0,n]) swap rows 'k' and 'l' on matrix A(_xn) */
void  mtxi_row_swp(int * A, const int n, int k, int l);
void  mtxf_row_swp(float * A, const int n, int k, int l);
void mtxlf_row_swp(double * A, const int n, int k, int l);
void mtxLf_row_swp(long double * A, const int n, int k, int l);

/*  A(l,[0,n]) += A(k,[0,n])	add row 'k' to row 'l' on matrix A(_xn) */
void  mtxi_row_add(int * A, const int n, int k, int l);
void  mtxf_row_add(float * A, const int n, int k, int l);
void mtxlf_row_add(double * A, const int n, int k, int l);
void mtxLf_row_add(long double * A, const int n, int k, int l);

/*  A(l,[0,n]) += x*A(k,[0,n])	add 'x' times row 'k' to row 'l' on matrix A(_xn) */
void  mtxi_row_adm(int * A, const int n, const int x, int k, int l);
void  mtxf_row_adm(float * A, const int n, const float x, int k, int l);
void mtxlf_row_adm(double * A, const int n, const double x, int k, int l);
void mtxLf_row_adm(long double * A, const int n, const long double x, int k, int l);

/* A(l,[0,n]) -= A(k,[0,n])	subtract row 'k' from row 'l' on matrix A(_xn) */
void  mtxi_row_sub(int * A, const int n, int k, int l);
void  mtxf_row_sub(float * A, const int n, int k, int l);
void mtxlf_row_sub(double * A, const int n, int k, int l);
void mtxLf_row_sub(long double * A, const int n, int k, int l);

/* A(l,[0,n]) += k	add scalar 'k' to row 'l' on matrix A(_xn) */
void  mtxi_row_add_scl(int * A, const int n, const int k, int l);
void  mtxf_row_add_scl(float * A, const int n, const float k, int l);
void mtxlf_row_add_scl(double * A, const int n, const double k, int l);
void mtxLf_row_add_scl(long double * A, const int n, const long double k, int l);

/* A(l,[0,n]) -= k	subtract scalar 'k' from row 'l' on matrix A(_xn) */
void  mtxi_row_sub_scl(int * A, const int n, const int k, int l);
void  mtxf_row_sub_scl(float * A, const int n, const float k, int l);
void mtxlf_row_sub_scl(double * A, const int n, const double k, int l);
void mtxLf_row_sub_scl(long double * A, const int n, const long double k, int l);

/* A(l,[0,n]) *= k	multiply row 'l' by scalar 'k' on matrix A(_xn) */
void  mtxi_row_mul_scl(int * A, const int n, const int k, int l);
void  mtxf_row_mul_scl(float * A, const int n, const float k, int l);
void mtxlf_row_mul_scl(double * A, const int n, const double k, int l);
void mtxLf_row_mul_scl(long double * A, const int n, const long double k, int l);

/* A(l,[0,n]) /= k	divide row 'l' by scalar 'k' on matrix A(_xn) */
void  mtxi_row_div_scl(int * A, const int n, const int k, int l);
void  mtxf_row_div_scl(float * A, const int n, const float k, int l);
void mtxlf_row_div_scl(double * A, const int n, const double k, int l);
void mtxLf_row_div_scl(long double * A, const int n, const long double k, int l);

/* A([0,m],l) <-> A([0,m],k) swap columns 'k' and 'l' on matrix A(mx_) */
void  mtxi_col_swp(int * A, const int m, int k, int l);
void  mtxf_col_swp(float * A, const int m, int k, int l);
void mtxlf_col_swp(double * A, const int m, int k, int l);
void mtxLf_col_swp(long double * A, const int m, int k, int l);

/*  A([0,m],l) += A([0,m],k)	add column 'k' to column 'l' on matrix A(mx_) */
void  mtxi_col_add(int * A, const int m, int k, int l);
void  mtxf_col_add(float * A, const int m, int k, int l);
void mtxlf_col_add(double * A, const int m, int k, int l);
void mtxLf_col_add(long double * A, const int m, int k, int l);

/*  A([0,m],l) += A([0,m],k)	adds 'x' times column 'k' to column 'l' on matrix A(mx_) */
void  mtxi_col_adm(int * A, const int n, const int x, int k, int l);
void  mtxf_col_adm(float * A, const int n, const float x, int k, int l);
void mtxlf_col_adm(double * A, const int n, const double x, int k, int l);
void mtxLf_col_adm(long double * A, const int n, const long double x, int k, int l);

/*  A([0,m],l) -= A([0,m],k)	subtract column 'k' from column 'l' on matrix A(mx_) */
void  mtxi_col_sub(int * A, const int m, int k, int l);
void  mtxf_col_sub(float * A, const int m, int k, int l);
void mtxlf_col_sub(double * A, const int m, int k, int l);
void mtxLf_col_sub(long double * A, const int m, int k, int l);

/* A([0,m],l) += k	add scalar 'k' to column 'l' on matrix A(mx_) */
void  mtxi_col_add_scl(int * A, const int m, const int k, int l);
void  mtxf_col_add_scl(float * A, const int m, const float k, int l);
void mtxlf_col_add_scl(double * A, const int m, const double k, int l);
void mtxLf_col_add_scl(long double * A, const int m, const long double k, int l);

/* A([0,m],l) -= k	subtract scalar 'k' from column 'l' on matrix A(mx_) */
void  mtxi_col_sub_scl(int * A, const int m, const int k, int l);
void  mtxf_col_sub_scl(float * A, const int m, const float k, int l);
void mtxlf_col_sub_scl(double * A, const int m, const double k, int l);
void mtxLf_col_sub_scl(long double * A, const int m, const long double k, int l);

/* A([0,m],l) *= k	multiply column 'l' by scalar 'k' on matrix A(mx_) */
void  mtxi_col_mul_scl(int * A, const int m, const int k, int l);
void  mtxf_col_mul_scl(float * A, const int m, const float k, int l);
void mtxlf_col_mul_scl(double * A, const int m, const double k, int l);
void mtxLf_col_mul_scl(long double * A, const int m, const long double k, int l);

/* A([0,m],l) \= k	divide column 'l' by scalar 'k' on matrix A(mx_) */
void  mtxi_col_div_scl(int * A, const int m, const int k, int l);
void  mtxf_col_div_scl(float * A, const int m, const float k, int l);
void mtxlf_col_div_scl(double * A, const int m, const double k, int l);
void mtxLf_col_div_scl(long double * A, const int m, const long double k, int l);

#define mtx_rox(q, C) \
	_Generic((C),\
	float*			:  mtxf_rox,\
	double*			: mtxlf_rox,\
	long double*		: mtxLf_rox\
	)((q),(C))

#define mtx_roy(q, C) \
	_Generic((C),\
	float*			:  mtxf_roy,\
	double*			: mtxlf_roy,\
	long double*		: mtxLf_roy\
	)((q),(C))

#define mtx_roz(q, C) \
	_Generic((C),\
	float*			:  mtxf_roz,\
	double*			: mtxlf_roz,\
	long double*		: mtxLf_roz\
	)((q),(C))

#define mtx_rot_xyz_etr(ade, A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_xyz_etr,\
	double*			: mtxlf_rot_xyz_etr,\
	long double*		: mtxLf_rot_xyz_etr\
	)((ade),(A),(C))

#define mtx_rot_xyz_rte(ade, A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_xyz_rte,\
	double*			: mtxlf_rot_xyz_rte,\
	long double*		: mtxLf_rot_xyz_rte\
	)((ade),(A),(C))

#define mtx_rot_xyz_etq(ade, A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_xyz_etq,\
	double*			: mtxlf_rot_xyz_etq,\
	long double*		: mtxLf_rot_xyz_etq\
	)((ade),(A),(C))

#define mtx_rot_xyz_qte(ade, A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_xyz_qte,\
	double*			: mtxlf_rot_xyz_qte,\
	long double*		: mtxLf_rot_xyz_qte\
	)((ade),(A),(C))

#define mtx_rot_xyz_cpm(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_xyz_cpm,\
	double*			: mtxlf_rot_xyz_cpm,\
	long double*		: mtxLf_rot_xyz_cpm\
	)((A),(C))

#define mtx_rot_xyz_cp2(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_xyz_cp2,\
	double*			: mtxlf_rot_xyz_cp2,\
	long double*		: mtxLf_rot_xyz_cp2\
	)((A),(C))

/* C(4x4) will be changed to a rotation matrix the respective axis */
void  mtxf_rox(const float q, float *C);
void  mtxf_roy(const float q, float *C);
void  mtxf_roz(const float q, float *C);
void mtxlf_rox(const double q, double *C);
void mtxlf_roy(const double q, double *C);
void mtxlf_roz(const double q, double *C);
void mtxLf_rox(const long double q, long double *C);
void mtxLf_roy(const long double q, long double *C);
void mtxLf_roz(const long double q, long double *C);
/* A(3x1) C(3x)	The cross product matrix C(w) */
void  mtxf_rot_xyz_cpm(const float * const A,  float *C);
void mtxlf_rot_xyz_cpm(const double * const  A, double *C);
void mtxLf_rot_xyz_cpm(const long double * const A, long double *C);
/* A(3x1) C(3x)	The cross product matrix squared C(w)2 */
void  mtxf_rot_xyz_cp2(const float * const A,  float *C);
void mtxlf_rot_xyz_cp2(const double * const  A, double *C);
void mtxLf_rot_xyz_cp2(const long double * const A, long double *C);

#define MTX_ROT_RAD 0
#define MTX_ROT_DEG 1
/* Conversions must be estecified whter to or from degrees */
void  mtxf_rot_xyz_etr(const int ade, const float * const A,  float *C);
void mtxlf_rot_xyz_etr(const int ade, const double * const  A, double *C);
void mtxLf_rot_xyz_etr(const int ade, const long double * const A, long double *C);

void  mtxf_rot_xyz_rte(const float * const A, const int cde, float *C);
void mtxlf_rot_xyz_rte(const double * const  A, const int cde,double *C);
void mtxLf_rot_xyz_rte(const long double * const A, const int cde,long double *C);

void  mtxf_rot_xyz_etq(const int ade, const float * const A,  float *C);
void mtxlf_rot_xyz_etq(const int ade, const double * const  A, double *C);
void mtxLf_rot_xyz_etq(const int ade, const long double * const A, long double *C);

void  mtxf_rot_xyz_qte(const float * const A, const int cde, float *C);
void mtxlf_rot_xyz_qte(const double * const  A, const int cde,double *C);
void mtxLf_rot_xyz_qte(const long double * const A, const int cde,long double *C);




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

#include <stdio.h>
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


#endif /* MPU_MTX_H_ */

#endif /* __STDC_VERSION__ >= 201112L */

#ifdef __cplusplus
	}
#endif /* __cplusplus */
