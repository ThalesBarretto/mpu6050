#ifdef __cplusplus
	extern "C" {
#endif

/* C11 or later required for type-generic macros */
#if __STDC_VERSION__ >= 201112L

#ifndef MPU_MTX_H_
#define MPU_MTX_H_
/*
 * MATRIX OPERATIONS - Generic
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
 * mtx_zer		Matrix Zero
 * mtx_eye		Matrix Identity
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

#endif /* MPU_MTX_H_ */

#endif /* __STDC_VERSION__ >= 201112L */

#ifdef __cplusplus
	}
#endif /* __cplusplus */
