#ifdef __cplusplus
	extern "C" {
#endif

/* C11 or later required for type-generic macros */
#if __STDC_VERSION__ >= 201112L
#ifndef MPU_MTX_ROT_XYZ_H_
#define MPU_MTX_ROT_XYZ_H_

#include "dev_mpu_mtx.h"
#include "dev_mpu_mtx_rot.h"

#define mtx_rot_xyz_etr(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_xyz_etr,\
	double*			: mtxlf_rot_xyz_etr,\
	long double*		: mtxLf_rot_xyz_etr\
	)((ade),(A),(C))

#define mtx_rot_xyz_rte(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_xyz_rte,\
	double*			: mtxlf_rot_xyz_rte,\
	long double*		: mtxLf_rot_xyz_rte\
	)((A),(C))

#define mtx_rot_xyz_etq(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_xyz_etq,\
	double*			: mtxlf_rot_xyz_etq,\
	long double*		: mtxLf_rot_xyz_etq\
	)(A),(C)

#define mtx_rot_xyz_qte(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_xyz_qte,\
	double*			: mtxlf_rot_xyz_qte,\
	long double*		: mtxLf_rot_xyz_qte\
	)((A),(C))

void  mtxf_rot_xyz_etr(const float 	 * const A, float 	*C);
void mtxlf_rot_xyz_etr(const double 	 * const A, double 	*C);
void mtxLf_rot_xyz_etr(const long double * const A, long double *C);

void  mtxf_rot_xyz_rte(const float 	 * const A, float 	*C);
void mtxlf_rot_xyz_rte(const double 	 * const A, double 	*C);
void mtxLf_rot_xyz_rte(const long double * const A, long double *C);

void  mtxf_rot_xyz_etq(const float 	 * const A, float 	*C);
void mtxlf_rot_xyz_etq(const double	 * const A, double 	*C);
void mtxLf_rot_xyz_etq(const long double * const A, long double *C);

void  mtxf_rot_xyz_qte(const float	 * const A, float	*C);
void mtxlf_rot_xyz_qte(const double 	 * const A, double	*C);
void mtxLf_rot_xyz_qte(const long double * const A, long double *C);

#endif /* MPU_MTX_ROT_XYZ_H_ */

#endif /* __STDC_VERSION__ >= 201112L */

#ifdef __cplusplus
	}
#endif /* __cplusplus */
