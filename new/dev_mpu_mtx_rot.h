#ifdef __cplusplus
	extern "C" {
#endif

#if __STDC_VERSION__ >= 201112L

#ifndef MPU_MTX_ROT_H_
#define MPU_MTX_ROT_H_

#include <math.h>

#define mtx_rox(q, C) _Generic((C),		\
	float*		:  mtxf_rox,		\
	double*		: mtxlf_rox,		\
	long double*	: mtxLf_rox		\
	)((q), (C))

#define mtx_roy(q, C) _Generic((C),		\
	float*		:  mtxf_roy,		\
	double*		: mtxlf_roy,		\
	long double*	: mtxLf_roy		\
	)((q), (C))

#define mtx_roz(q, C) _Generic((C),		\
	float*		:  mtxf_roz,		\
	double*		: mtxlf_roz,		\
	long double*	: mtxLf_roz		\
	)((q), (C))

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

#endif /* __STDC_VERSION__ >= 201112L */

/* MPU_MTX_ROT_H */
#ifdef __cplusplus
	}
#endif
