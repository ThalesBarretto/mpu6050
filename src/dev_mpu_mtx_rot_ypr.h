#ifdef __cplusplus
	extern "C" {
#endif

/* C11 or later required for type-generic macros */
#if __STDC_VERSION__ >= 201112L
#ifndef MPU_MTX_ROT_YPR_H_
#define MPU_MTX_ROT_YPR_H_

/* this is part of dev_mpu_mtx_rot */
#include "dev_mpu_mtx_rot.h"

#define mtx_rot_ypr_etr(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_ypr_etr,\
	double*			: mtxlf_rot_ypr_etr,\
	long double*		: mtxLf_rot_ypr_etr\
	)((ade),(A),(C))

#define mtx_rot_ypr_etq(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_ypr_etq,\
	double*			: mtxlf_rot_ypr_etq,\
	long double*		: mtxLf_rot_ypr_etq\
	)((A),(C))
#define mtx_rot_ypr_rte(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_ypr_rte,\
	double*			: mtxlf_rot_ypr_rte,\
	long double*		: mtxLf_rot_ypr_rte\
	)((A),(C))
#define mtx_rot_ypr_rtq(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_ypr_rtq,\
	double*			: mtxlf_rot_ypr_rtq,\
	long double*		: mtxLf_rot_ypr_rtq\
	)((A),(C))
#define mtx_rot_ypr_qte(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_ypr_qte,\
	double*			: mtxlf_rot_ypr_qte,\
	long double*		: mtxLf_rot_ypr_qte\
	)((A),(C))
#define mtx_rot_ypr_qtr(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_ypr_qtr,\
	double*			: mtxlf_rot_ypr_qtr,\
	long double*		: mtxLf_rot_ypr_qtr\
	)((A),(C))
#define mtx_rot_ypr_wtp(A, B, C) \
	_Generic((C),\
	float*			:  mtxf_rot_ypr_wtp,\
	double*			: mtxlf_rot_ypr_wtp,\
	long double*		: mtxLf_rot_ypr_wtp\
	)((A), (B), (C))

#define mtx_rot_ypr_btr(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_ypr_btr,\
	double*			: mtxlf_rot_ypr_btr,\
	long double*		: mtxLf_rot_ypr_btr\
	)((ade),(A),(C))

#define mtx_rot_ypr_btq(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_ypr_btq,\
	double*			: mtxlf_rot_ypr_btq,\
	long double*		: mtxLf_rot_ypr_btq\
	)((A),(C))
#define mtx_rot_ypr_bte(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_ypr_bte,\
	double*			: mtxlf_rot_ypr_bte,\
	long double*		: mtxLf_rot_ypr_bte\
	)((A),(C))

void  mtxf_rot_ypr_etr(const float 	 * const A, float 	*C);
void mtxlf_rot_ypr_etr(const double 	 * const A, double 	*C);
void mtxLf_rot_ypr_etr(const long double * const A, long double *C);

void  mtxf_rot_ypr_etq(const float 	 * const A, float 	*C);
void mtxlf_rot_ypr_etq(const double	 * const A, double 	*C);
void mtxLf_rot_ypr_etq(const long double * const A, long double *C);

void  mtxf_rot_ypr_rte(const float 	 * const A, float 	*C);
void mtxlf_rot_ypr_rte(const double 	 * const A, double 	*C);
void mtxLf_rot_ypr_rte(const long double * const A, long double *C);

void  mtxf_rot_ypr_rtq(const float 	 * const A, float 	*C);
void mtxlf_rot_ypr_rtq(const double 	 * const A, double 	*C);
void mtxLf_rot_ypr_rtq(const long double * const A, long double *C);

void  mtxf_rot_ypr_qte(const float	 * const A, float	*C);
void mtxlf_rot_ypr_qte(const double 	 * const A, double	*C);
void mtxLf_rot_ypr_qte(const long double * const A, long double *C);

void  mtxf_rot_ypr_qtr(const float	 * const A, float	*C);
void mtxlf_rot_ypr_qtr(const double 	 * const A, double	*C);
void mtxLf_rot_ypr_qtr(const long double * const A, long double *C);

void  mtxf_rot_ypr_wtp(const float	 * const A, const float	 	 * const B, float	*C);
void mtxlf_rot_ypr_wtp(const double 	 * const A, const double 	 * const B, double	*C);
void mtxLf_rot_ypr_wtp(const long double * const A, const long double 	 * const B, long double *C);

void  mtxf_rot_ypr_btr(const float 	 * const A, float 	*C);
void mtxlf_rot_ypr_btr(const double 	 * const A, double 	*C);
void mtxLf_rot_ypr_btr(const long double * const A, long double *C);

void  mtxf_rot_ypr_btq(const float 	 * const A, float 	*C);
void mtxlf_rot_ypr_btq(const double	 * const A, double 	*C);
void mtxLf_rot_ypr_btq(const long double * const A, long double *C);

void  mtxf_rot_ypr_bte(const float 	 * const A, float 	*C);
void mtxlf_rot_ypr_bte(const double 	 * const A, double 	*C);
void mtxLf_rot_ypr_bte(const long double * const A, long double *C);
#endif /* MPU_MTX_ROT_YPR_H_ */

#endif /* __STDC_VERSION__ >= 201112L */

#ifdef __cplusplus
	}
#endif /* __cplusplus */
