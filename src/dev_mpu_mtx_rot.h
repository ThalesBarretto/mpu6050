#ifdef __cplusplus
	extern "C" {
#endif
#if __STDC_VERSION__ >= 201112L
#ifndef MPU_MTX_ROT_H_
#define MPU_MTX_ROT_H_

/* this is part of dev_mpu_mtx */
#include "dev_mpu_mtx.h"

/* needed for all calculations */
#include <math.h>
#include <tgmath.h>

/*
 * MATRIX ROTATIONS - Generic 
 * 	= A,B are sources, C is destination
 * 	= Type chosen by C, do not mix
 * 	= Operands must not overlap
 * 	= no checks performed
 * 	+ Angles in RADIANS
 * 	+ All rotations are right-handend, unless otherwise specified
 * 	+ Axis are X,Y,Z, unless otherwise specified
 * 	+ Matrices     are 3x3 [r11 r12 r13; r21 r22 r23; r31 r32 r33]
 * 	+ Euler angles are 3x1 [theta1; theta2; theta3]
 * 	+ Quaternions  are 4x1 [q0; q1; q2; q3]
 *
 * Helpers:
 * 	PI: 	constant long double 
 * 	r2d(q)	radians to degrees
 *	d2r(q)	degrees to radians
 *	squ(x)	square squ(x)=x²
 */
#define mtx_rot_rvx(q, C) \
	_Generic((C),\
	float*			:  mtxf_rot_rvx,\
	double*			: mtxlf_rot_rvx,\
	long double*		: mtxLf_rot_rvx\
	)((q),(C))

#define mtx_rot_rvy(q, C) \
	_Generic((C),\
	float*			:  mtxf_rot_rvy,\
	double*			: mtxlf_rot_rvy,\
	long double*		: mtxLf_rot_rvy\
	)((q),(C))

#define mtx_rot_rvz(q, C) \
	_Generic((C),\
	float*			:  mtxf_rot_rvz,\
	double*			: mtxlf_rot_rvz,\
	long double*		: mtxLf_rot_rvz\
	)((q),(C))

#define mtx_rot_rfx(q, C) \
	_Generic((C),\
	float*			:  mtxf_rot_rfx,\
	double*			: mtxlf_rot_rfx,\
	long double*		: mtxLf_rot_rfx\
	)((q),(C))

#define mtx_rot_rfy(q, C) \
	_Generic((C),\
	float*			:  mtxf_rot_rfy,\
	double*			: mtxlf_rot_rfy,\
	long double*		: mtxLf_rot_rfy\
	)((q),(C))

#define mtx_rot_rfz(q, C) \
	_Generic((C),\
	float*			:  mtxf_rot_rfz,\
	double*			: mtxlf_rot_rfz,\
	long double*		: mtxLf_rot_rfz\
	)((q),(C))

#define mtx_rot_cpm(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_cpm,\
	double*			: mtxlf_rot_cpm,\
	long double*		: mtxLf_rot_cpm\
	)((A),(C))
#define mtx_rot_cp2(A, C) \
	_Generic((C),\
	float*			:  mtxf_rot_cp2,\
	double*			: mtxlf_rot_cp2,\
	long double*		: mtxLf_rot_cp2\
	)((A),(C))

/* Use C(3x3)*v to rotate vector 'v' around axis by angle q */
void  mtxf_rot_rvx(const float 	 	q, float	*C);
void  mtxf_rot_rvy(const float 	 	q, float	*C);
void  mtxf_rot_rvz(const float 	 	q, float	*C);
void mtxlf_rot_rvx(const double	 	q, double	*C);
void mtxlf_rot_rvy(const double	 	q, double	*C);
void mtxlf_rot_rvz(const double	 	q, double	*C);
void mtxLf_rot_rvx(const long double	q, long double	*C);
void mtxLf_rot_rvy(const long double	q, long double	*C);
void mtxLf_rot_rvz(const long double	q, long double	*C);

/* Use C(3x3)*F to rotate a frame 'F' around axis by angle q */
void  mtxf_rot_rfx(const float 	 	q, float	*C);
void  mtxf_rot_rfy(const float 	 	q, float	*C);
void  mtxf_rot_rfz(const float 	 	q, float	*C);
void mtxlf_rot_rfx(const double	 	q, double	*C);
void mtxlf_rot_rfy(const double	 	q, double	*C);
void mtxlf_rot_rfz(const double	 	q, double	*C);
void mtxLf_rot_rfx(const long double	q, long double	*C);
void mtxLf_rot_rfy(const long double	q, long double	*C);
void mtxLf_rot_rfz(const long double	q, long double	*C);

/* A(3x1) C(3x3), The cross product matrix C(w) */
void  mtxf_rot_cpm(const float 		* const A, float	*C);
void mtxlf_rot_cpm(const double 	* const A, double	*C);
void mtxLf_rot_cpm(const long double    * const A, long double	*C);
/* A(3x1) C(3x3), The cross product matrix squared C(w)² */
void  mtxf_rot_cp2(const float 		* const A, float	*C);
void mtxlf_rot_cp2(const double 	* const A, double	*C);
void mtxLf_rot_cp2(const long double    * const A, long double	*C);

/* helpers */
#ifndef PI
#ifdef M_PI
#define PI M_PI
#else
#define PI (3.1415926535897932384)
#define M_PI PI
#endif
#endif

#define squ(x) ((x)*(x))
static const float  		mtx_rot_r2df	= (float)(180./M_PI);
static const double  		mtx_rot_r2dl 	= 	 (180./M_PI);
static const long double	mtx_rot_r2dLf	= 	(180.L/M_PI);
static const float  		mtx_rot_d2rf	= (float)(M_PI/180.);
static const double  		mtx_rot_d2rl 	= 	 (M_PI/180. );
static const long double	mtx_rot_d2rLf	= 	 (M_PI/180.L);

#define r2d(q)  _Generic((q),	 \
	float		:  r2df, \
	double		: r2dlf, \
	long double	: r2dLf  \
	)(q)

#define d2r(q) _Generic((q),	 \
	float		:  d2rf, \
	double		: d2rlf, \
	long double	: d2rLf  \
	)(q)

static inline float   __attribute__((always_inline)) d2rf(const float q)
{
	return q * mtx_rot_d2rf;
}
static inline double   __attribute__((always_inline)) d2rlf(const double q)
{
	return q * mtx_rot_d2rl;
}
static inline long double   __attribute__((always_inline)) d2rLf(const long double q)
{
	return q * mtx_rot_d2rLf;
}

static inline float   __attribute__((always_inline)) r2df(const float q)
{
	return q * mtx_rot_r2df;
}
static inline double   __attribute__((always_inline)) r2dlf(const double q)
{
	return q * mtx_rot_r2dl;
}

static inline long double   __attribute__((always_inline)) r2dLf(const long double q)
{
	return q * mtx_rot_r2dLf;
}

#endif /* MPU_MTX_ROT_H_ */

#endif /* __STDC_VERSION__ >= 201112L */

#ifdef __cplusplus
	}
#endif /* __cplusplus */

/* NOTES
 * 	-Rotate a vector 'x' by 'theta' around 'Z' to get vector 'y':
 * 		R_Z =	[[	cos(theta)		-sin(theta)		0]
 * 		   	 [	sin(theta)		cos(theta)		0]
 * 		   	 [	0			0			1]];
 * 		y = R_Z x
 *
 * 	-Rotate a frame 'A' by 'theta' around 'Z' to get frame 'B':
 * 		v^{a} = C_b/a v^{b}
 * 		C_b/a =	[[	cos(theta)		sin(theta)		0]
 * 		   	 [	-sin(theta)		cos(theta)		0]
 * 		   	 [	0			0			1]];
 *
 * 	-Vector v^a expressed in frame 'A' can be expressed as v^b in frame 'B' by:
 * 		V^b = C_a/b V^a
 *
 * 	-Vector v^b expressed in frame 'B' can be expressed as v^a in frame 'A' by:
 * 		C_b/a =	 {R_Z^T}^T = R_Z;
 * 		V^a = C_a/b V^b
 *
 *	TODO: FIXME
 * 	-Sequential rotations of a vector are around incrementally primed axes:
 * 		R_Z around Z	by theta1
 * 		R_Y around Y'	by theta2
 * 		R_X around X''	by theta3
 * 		or, simply: R_XYZ(1,2,3) = RZ(theta3) * RY(theta2) * RX(theta1)
 * 		
 * 	-Reverse rotations C_b/a is the reverse of C_a/b
 * 		C_a/b = RZ(theta3)     * RY(theta2)    * RX(theta1)
 * 		C_b/a = RX(theta1)^{T} * RY(theta2)^{T}* RZ(theta3)^{T}
 *
 * Functions (for each sequence XYZ, YZX, ZYX ...):
 * 	Type:
 *
 * 	'R' rotates a vector by sequence xyz(123);
 * 		R_xyz(1,2,3): v^{ref}_after  = Rz(q3) * Ry(q2) * Rx(q1) * v^{ref}_before
 * 		Rx(q1)=	[[ 1		0		0	];
 * 		   	 [ 0		cos(q1)		sin(q1) ];
 * 		   	 [ 0		-sin(q1)	cos(q1)	]];
 * 		Ry(q2)=	[[ cos(q2)	0		-sin(q2)];
 * 		   	 [ 0		1		0	];
 * 		   	 [ sin(q2)	0		cos(q2)	]];
 * 		Rz(q3)=	[[ cos(q3)	sin(q3)		0	];
 * 		   	 [ -sin(q3)	cos(q3)		0	];
 * 		   	 [ 0		0		1	]];
 * 		
 *
 * 	'M' transforms a vector expressed in 'body' frame to vectors expressed in 'ref' frame
 * 		M_xyz(1,2,3): v^{ref} = Mx(q1)*My(q2)*Mz(q3)*v^{body}
 * 		Mx(q1)=	[[ 1		0		0	];
 * 		   	 [ 0		cos(q1)		-sin(q1)];
 * 		   	 [ 0		sin(q1)		cos(q1)	]];
 * 		My(q2)=	[[ cos(q2)	0		sin(q2)	];
 * 		   	 [ 0		1		0	];
 * 		   	 [ -sin(q2)	0		cos(q2)	]];
 * 		Mz(q3)=	[[ cos(q3)	-sin(q3)	0	];
 * 		   	 [ sin(q3)	cos(q3)		0	];
 * 		   	 [ 0		0		1	]];
 *
 * 	-C: v^b as v^r, with C = C_b/r = Rz * Ry * Rx
 *
 * 	1.ETR - Find the rotation matrix, given 3 Euler angles
 * 	3.ETQ - Find the unit quaternion, given Euler angles
 * 	2.RTE - Find the Euler angles, given a rotation matrix
 *
 * 	e.g., for the sequence XYZ(1,2,3) 
 * 		float V1[3] = {1,1,1};
 * 		float E[3] = {theta1, theta2, theta3};
 *
 * 		float V2[3]};
 * 		float M[3][3];
 * 		float C[3][3];
 *
 * 		R_xyz(E, V1, V2);
 * 		M_xyz(E, C);
 * 		C_xyz(E, C);
 */
