#ifdef __cplusplus
	extern "C" {
#endif

/* C11 or later required for type-generic macros */
#if __STDC_VERSION__ >= 201112L
#ifndef MPU_MTX_ROT_H_
#define MPU_MTX_ROT_H_
/*
 * MATRIX ROTATIONS
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
 * Rotation Matrices:
 * 	- given angles unit must chosen to be RADS or DEGS
 * 	- returned angles unit must chosen to be RADS or DEGS
 * 	- Rotation Matrices are (4x4)
 * 	- Euler angles are (3x1)[phi, theta, psi]
 * 	- Quaterions are   (4x1)[q0, q1, q2, q3]
 * 	- *A is always the source
 * 	- *C is always the destination
 * 	- no checks performed
 */

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

#endif /* MPU_MTX_ROT_H_ */

#endif /* __STDC_VERSION__ >= 201112L */

#ifdef __cplusplus
	}
#endif /* __cplusplus */
