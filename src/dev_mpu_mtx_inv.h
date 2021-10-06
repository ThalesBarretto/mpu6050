#ifdef __cplusplus
	extern "C" {
#endif

/* C11 or later required for type-generic macros */
#if __STDC_VERSION__ >= 201112L
#ifndef MPU_MTX_INV_H_
#define MPU_MTX_INV_H_

/* This is part of dev_mpu_mtx */
#include "dev_mpu_mtx.h"

/*
 * MATRIX INVERSION
 * 	- YOU MUST PROVIDE A BUFFER "B"
 * 	- A is the source (not modified)
 * 	- B is the buffer (overwritten)
 * 	- C is the destination (C = A_inverse)
 * 	- n is the size of A(nxn) and C(nxn)
 * 	- no checks performed (you must provide an invertible matrix)
 * 	- only fails when there is a zero on the pivot
 * 	- returns 0 on success, -1 on failure
 *
 * Functions:
 * 	mtx_inv		Matrix Inversion
 */

#define mtx_inv(A, B, n, C)			\
	_Generic((C),				\
	int*		:  mtxi_inv,	 	\
	float*		:  mtxf_inv,	 	\
	double*		: mtxlf_inv,	 	\
	long double*	: mtxLf_inv	 	\
	)((A), (B), (n), (C))

/* C(mxn) = A(mxn)_inverse */
int  mtxi_inv(int 		*A, int		 *B, const int n, int		*C);
int  mtxf_inv(float 		*A, float	 *B, const int n, float		*C);
int mtxlf_inv(double		*A, double	 *B, const int n, double	*C);
int mtxLf_inv(long double	*A, long double	 *B, const int n, long double	*C);

#endif /* MPU_MTX_INV_H_ */

#endif /* __STDC_VERSION__ >= 201112L */

#ifdef __cplusplus
	}
#endif /* __cplusplus */
