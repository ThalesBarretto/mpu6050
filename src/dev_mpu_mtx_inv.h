#ifdef __cplusplus
	extern "C" {
#endif

/* C11 or later required for type-generic macros */
#if __STDC_VERSION__ >= 201112L
#ifndef MPU_MTX_INV_H_
#define MPU_MTX_INV_H_
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
 * mtx_inv		Matrix Inversion
 */

#define mtx_inv(A, B, n, C)			\
	_Generic((C),				\
	int*		:  mtxi_inv,	 	\
	float*		:  mtxf_inv,	 	\
	double*		: mtxlf_inv,	 	\
	long double*	: mtxLf_inv	 	\
	)((A), (B), (n), (C))

/* C(mxn) = A(mxn)_inverse */
int  mtxi_inv(int *  A, int * B, const int n, int *C);
int  mtxf_inv(float * A, float * B, const int n, float *C);
int mtxlf_inv(double * A, double * B, const int n, double *C);
int mtxLf_inv(long double * A, long double * B, const int n, long double *C);

#endif /* MPU_MTX_INV_H_ */

#endif /* __STDC_VERSION__ >= 201112L */

#ifdef __cplusplus
	}
#endif /* __cplusplus */
