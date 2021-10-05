#include "dev_mpu_mtx.h"
#include "dev_mpu_mtx_rot.h"
#include <math.h>
#include <tgmath.h>
/* unit converion in extra precision */
#define PI (3.1415926535897932384L)
#define r2d(q) ((q)*(180.0L/PI)) 
#define d2r(q) ((q)*(PI/180.0L)) 

#define MTX_ROX \
	c[0][0] =	1;\
	c[0][1] =	0;\
	c[0][2] =	0;\
	c[1][0] =	0;\
	c[1][1] =	cq;\
	c[1][2] =	sq;\
	c[2][0] =	0;\
	c[2][1] =	-sq;\
	c[2][2] =	cq;\

#define MTX_ROY \
	c[0][0] =	cq;\
	c[0][1] =	0;\
	c[0][2] =	sq;\
	c[1][0] =	0;\
	c[1][1] =	1;\
	c[1][2] =	0;\
	c[2][0] =	-sq;\
	c[2][1] =	0;\
	c[2][2] =	cq;\

#define MTX_ROZ \
	c[0][0] =	cq;\
	c[0][1] =	sq;\
	c[0][2] =	0;\
	c[1][0] =	-sq;\
	c[1][1] =	cq;\
	c[1][2] =	0;\
	c[2][0] =	0;\
	c[2][1] =	0;\
	c[2][2] =	1;\

/*
 * Given a sequence of axis, rotates sequentially by given angles (in radians). 
 * C(4x4) will become the matrix that implements given rotation
 * -rot_zyx = Rx*Ry*Rz (rotate about z, about y then about x)
 * -rot_xyz = Rz*Ry*Rx (rotate about x, about y then about z)
 */ 

/* SOURCES:
 *
 * Freescale Semiconductor Application Note AN3461 Rev. 6, 03/2013 - Tilt
 * Sensing Using a Three-Axis Accelerometer, by Mark Pedley
 *
 * DIEBEL, James - Representing Attitude Euler Angles, unit quaternions and
 * rotation vectores. Stanford (2006). p.24 (xyz), p.33 (zyx).
 *
 * 5.6.1 The angles associated with sequence XYZ (1,2,3) are called Cardan angles,
 * Tait-Bryan angles or nautical angles. In aerospace engineering and computer
 * graphics they are often callen roll, pitch, yaw or bank, attitude, heading.
 *
 * These angles describe a vehicle whose forward direction is along the positive
 * body-fixed x-axis, with the body-fixed y-axis to starboard, and th body-fixed
 * z-axis downward. In such a configuration, the home position [phi, theta, psi]
 * = [0 0 0] is flat level, pointing forward along the world x-axis.
 *
 * The non-intuitive downward-pointing z-axis is chosen in order to make a
 * positive change in theta correspond to pitching upward. A less common
 * standard using the same sequence is to have the y-axis point to port and the
 * z-axis point upward. In this case, a positive change in theta correspond to
 * pitching downwward. ...
 *
 * 5.6.2. The function that maps a vector of Euler angles to its rotation matrix
 * ...:
 * R123(phi, the, psi) = R1(phi) * R2(the) * R3(psi) = MTX_ROT_XYZ
 *
 * 5.6.3. The inverse mapping, which gives th Euler angles as a function of the
 * rotation matrix, and the composition of that function with the rotation
 * matrix are:
 *
 * ...
 * 6.1.
 * A quaternion may be expressed as a vector
 * q = [q0, q1, q2, q3]T = [q0 ; q1:3]
 * its adjoint = [q0 ; -q1:3]
 * its norm 	= sqrt(q0² + q1² + q2² + q3²)
 * its inverse = adjoint / norm 
 * ...
 * 6.4.
 * Unit quaternions have unity norm ( = 1)
 * ...
 * 7.12 Integration of angular velocity
 * 		Quaternions are very well suited to tracking the attitude of an 
 * 	object by integrating the body-fixed angular velocity over time.
 * 		Consider an object with a body-fixed angular velocity of w'(t).
 * 	Let us consider the change in attitude from time t0 to time t1. We define
 * 	the rotation vector over this time interval to be:
 *
 * 		w'(to,t1) := integral[t0,t1](w'(t)dt)
 *
 * 		If the body-fixed angular velocity is provided as discrete samples,
 * 	as, for example, from a set of rate gyros, the integration will have to be
 * 	carried out numerically.
 *
 * 		If at time t0 the body has a quaternion attitude of q0, then the
 * 	attitude at time t1:
 *
 * 		q1 = [qv (circle) Vw'(t0,t1)] (dot) q0 =
 * 		q1 =  qv * Vw'(t0,t1) (dot) q0 
 *
 * 		Which con be generalized to:
 *
 * 		qi+1 =  qv * Vw'(t,t+1) (dot) qi 
 *
 *
 */

/* 
 * Find the cross-product-matrix C(w)[3x3] from gyro rates wx,xy,xz
 * (Eq. 23)
 */
#define MTX_ROT_XYZ_CPM		\
 	c[0][0] =  0;		\
 	c[0][1] = -A[2];	\
 	c[0][2] =  A[1];	\
 	c[1][0] =  A[2];	\
 	c[1][1] =  0;		\
 	c[1][2] = -A[0];	\
 	c[2][0] = -A[1];	\
 	c[2][1] =  A[0];	\
 	c[2][2] =  0;

/* 
 * Find cross-product-matrix square C(w)2[3x3] from gyro rates wx,xy,xz
 * (Eq. 27)
 */
#define MTX_ROT_XYZ_CP2				\
 	c[0][0] = -(A[2] * A[2]) -(A[1] * A[1])  ;	\
 	c[0][1] =  (A[1] * A[0] );			\
 	c[0][2] =  (A[2] * A[0]) ;			\
 	c[1][0] =  (A[1] * A[0]) ;			\
 	c[1][1] = -(A[2] * A[2]) -(A[0] * A[0])  ;	\
 	c[1][2] =  (A[2] * A[1]) ;			\
 	c[2][0] =  (A[2] * A[0]) ;			\
 	c[2][1] =  (A[2] * A[1]) ;			\
 	c[2][2] = -(A[1] * A[1]) -(A[0] * A[0]) ;	\

/* 
 * Find the Euler angles [phi, theta, psi] given a rotation matrix
 * (Eq. 72) u123(R) = [phi123(R) the123(R) psi123(R)] = 
 */

/*
 * #define MTX_ROT_XYZ_RTE						\
 *  	C[0] =   atan2(a[1][2], a[2][2]); 				\
 *  	C[1] =  -asin( a[0][2]);					\
 *  	C[2] =   atan2(a[0][1], a[0][0]); 
 * 	C[0] =   
 */
/*
 * The matrix from for body_xyz to angles en ned frame.
 * Notice the inversion of y and z do right and down (body frame)
 */
#define MTX_ROT_XYZ_RTE							\
 	C[0] =   atan2(-a[1][2], a[2][2]); 				\
 	C[1] =  -asin( a[0][2]);					\
 	C[2] =   atan2(-a[0][1], a[0][0]); 


/* Find the R(4x4) rotation matrix, given sequence 1,2,3(phi, the, psi) */
/* (Eq 287) */
/*
#define MTX_ROT_XYZ_ETR							\
	c[0][0] = ( c_the * c_psi);					\
	c[0][1] = ( c_the * s_psi);					\
	c[0][2] = (-s_the);						\
	c[0][3] = 0;							\
	c[1][0] = ( s_phi * s_the * c_psi) - ( c_phi * s_psi);		\
	c[1][1] = ( s_phi * s_the * s_psi) + ( c_phi * c_psi);		\
	c[1][2] = ( c_the * s_phi);					\
	c[1][3] = 0;							\
	c[2][0] = ( c_phi * s_the * c_psi) + ( s_phi * s_psi);		\
	c[2][1] = ( c_phi * s_the * s_psi) - ( s_phi * c_psi);		\
	c[2][2] = ( c_the * c_phi);					\
	c[2][3] = 0;							\
	c[3][0] = 0;							\
	c[3][1] = 0;							\
	c[3][2] = 0;							\
	c[3][3] = 1;
	*/
/* 3x3 roatation given yaw pitch roll */
#define MTX_ROT_XYZ_ETR							\
	c[0][0] = ( c_the * c_psi);					\
	c[0][1] = ( c_the * -s_psi);					\
	c[0][2] = (-s_the);						\
	c[1][0] = ( -s_phi * s_the * c_psi) + ( c_phi * s_psi);		\
	c[1][1] = ( s_phi * s_the * s_psi) + ( c_phi * c_psi);		\
	c[1][2] = ( c_the * -s_phi);					\
	c[2][0] = ( c_phi * s_the * c_psi) + ( s_phi * s_psi);		\
	c[2][1] = ( s_phi * c_psi) - ( c_phi * s_the * s_psi) ;		\
	c[2][2] = ( c_the * c_phi);


/* 
 * Find the quaternion given Euler angles [roll, pitch, yaw]
 * (Eq. 84) q123(phi,the,psi) =  
 */
#define MTX_ROT_XYZ_ETQ							\
 	C[0] = ( cph * cth * cps) + (sph * sth * sps);			\
	C[1] = (-cph * sth * sps) + (cth * cps * sph);			\
 	C[2] = ( cph * cps * sth) + (sph * cth * sps);			\
	C[3] = ( cph * cth * sps) - (sph * cps * sth);

/* 
 * Find Euler angles [phi, theta, psi] given a quaternion rotation matrix
 * (Eq. 73) u123(Rq(q)) =  
 */
#define MTX_ROT_XYZ_QTE							\
 	C[0] =   atan2((2*A[2]*A[3]) + (2*A[0]*A[1]), 			\
	     (A[3]*A[3]) - (A[2]*A[2]) - (A[1]*A[1]) + (A[0]*A[0])); 	\
	C[1] =  -asin((2*A[1]*A[3]) - (2*A[0]*A[2]));			\
 	C[2] =   atan2((2*A[1]*A[2]) + (2*A[0]*A[3]), 			\
		(A[1]*A[1]) + (A[0]*A[0]) - (A[3]*A[3]) - (A[2]*A[2])); 

/* Find the (4x4) rotation matrix, given sequence 3,2,1(phi, the, psi) */
#define MTX_ROT_ZYX_ETR							\
	c[0][0] =   c_psi * c_the;					\
	c[0][1] = ( c_phi * s_psi) + (c_psi * s_phi * s_the);		\
	c[0][2] = ( s_phi * s_psi) - (c_phi * c_psi * s_the);		\
	c[1][0] = (-c_the * s_psi); 					\
	c[1][1] = ( c_psi * c_phi) - (s_the * s_phi * s_psi); 		\
	c[1][2] = ( c_psi * s_phi) + (c_phi * s_psi * s_the);		\
	c[2][0] =    s_the; 						\
	c[2][1] = (- c_the * s_phi);					\
	c[2][2] =    c_the * c_phi;					\

/* C(4x4)=I will be changed to cos(q),... in respective values */
void  mtxf_rox(const float q, float *C)
{
	float (*c)[3] = (float (*)[3])C;
	float cq = cos(q);
	float sq = sin(q);
	MTX_ROX;
}
void  mtxf_roy(const float q, float *C)
{
	float (*c)[3] = (float (*)[3])C;
	float cq = cos(q);
	float sq = sin(q);
	MTX_ROY;
}
void  mtxf_roz(const float q, float *C)
{
	float (*c)[3] = (float (*)[3])C;
	float cq = cos(q);
	float sq = sin(q);
	MTX_ROZ;
}
void mtxlf_rox(const double q, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	double cq = cosl(q);
	double sq = sinl(q);
	MTX_ROX;
}
void mtxlf_roy(const double q, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	double cq = cosl(q);
	double sq = sinl(q);
	MTX_ROY;
}
void mtxlf_roz(const double q, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	double cq = cos(q);
	double sq = sin(q);
	MTX_ROZ;
}
void mtxLf_rox(const long double q, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	long double cq = cosl(q);
	long double sq = sinl(q);
	MTX_ROX;
}
void mtxLf_roy(const long double q, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	long double cq = cosl(q);
	long double sq = sinl(q);
	MTX_ROY;
}
void mtxLf_roz(const long double q, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	long double cq = cosl(q);
	long double sq = sinl(q);
	MTX_ROZ;
}

void  mtxf_rot_xyz_cpm(const float * const A,  float *C)
{
	float (*c)[3] = (float (*)[3])C;
	MTX_ROT_XYZ_CPM;
}
void mtxlf_rot_xyz_cpm(const double * const  A, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	MTX_ROT_XYZ_CPM;
}
void mtxLf_rot_xyz_cpm(const long double * const A, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	MTX_ROT_XYZ_CPM;
}
void  mtxf_rot_xyz_cp2(const float * const A,  float *C)
{
	float (*c)[3] = (float (*)[3])C;
	MTX_ROT_XYZ_CP2;
}
void mtxlf_rot_xyz_cp2(const double * const  A, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	MTX_ROT_XYZ_CP2;
}
void mtxLf_rot_xyz_cp2(const long double * const A, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	MTX_ROT_XYZ_CP2;
}
void  mtxf_rot_xyz_etr(const int ade, const float * const A,  float *C)
{
	float (*c)[3] = (float (*)[3])C;
	float s_psi;
	float s_the;
	float s_phi;
	float c_psi;
	float c_the;
	float c_phi;
	if (ade) {
		s_psi = sinl(d2r(A[2]));
		s_the = sinl(d2r(A[1]));
		s_phi = sinl(d2r(A[0]));
		c_psi = cosl(d2r(A[2]));
		c_the = cosl(d2r(A[1]));
		c_phi = cosl(d2r(A[0]));
	} else {
		s_psi = sinf(A[2]);
		s_the = sinf(A[1]);
		s_phi = sinf(A[0]);
		c_psi = cosf(A[2]);
		c_the = cosf(A[1]);
		c_phi = cosf(A[0]);
	}
	MTX_ROT_XYZ_ETR;
}
void mtxlf_rot_xyz_etr(const int ade, const double * const  A, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	double s_psi;
	double s_the;
	double s_phi;
	double c_psi;
	double c_the;
	double c_phi;
	if (ade) {
		s_psi = sinl(d2r(A[2]));
		s_the = sinl(d2r(A[1]));
		s_phi = sinl(d2r(A[0]));
		c_psi = cosl(d2r(A[2]));
		c_the = cosl(d2r(A[1]));
		c_phi = cosl(d2r(A[0]));
	} else {
		s_psi = sin(A[2]);
		s_the = sin(A[1]);
		s_phi = sin(A[0]);
		c_psi = cos(A[2]);
		c_the = cos(A[1]);
		c_phi = cos(A[0]);
	}
	MTX_ROT_XYZ_ETR;
}
void mtxLf_rot_xyz_etr(const int ade, const long double * const A, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	long double s_psi;
	long double s_the;
	long double s_phi;
	long double c_psi;
	long double c_the;
	long double c_phi;
	if (ade) {
		s_psi = sinl(d2r(A[2]));
		s_the = sinl(d2r(A[1]));
		s_phi = sinl(d2r(A[0]));
		c_psi = cosl(d2r(A[2]));
		c_the = cosl(d2r(A[1]));
		c_phi = cosl(d2r(A[0]));
	} else {
		s_psi = sinl(A[2]);
		s_the = sinl(A[1]);
		s_phi = sinl(A[0]);
		c_psi = cosl(A[2]);
		c_the = cosl(A[1]);
		c_phi = cosl(A[0]);
	}
	MTX_ROT_XYZ_ETR;
}

void  mtxf_rot_xyz_rte(const float * const A, const int cde, float *C)
{
	float (*a)[3] = (float (*)[3])A;
	MTX_ROT_XYZ_RTE;
	if (cde) {
		C[2]= r2d(C[2]);
		C[1]= r2d(C[1]);
		C[0]= r2d(C[0]);
	}
}
void mtxlf_rot_xyz_rte(const double * const  A, const int cde,double *C)
{
	double (*a)[3] = (double (*)[3])A;
	MTX_ROT_XYZ_RTE;
	if (cde) {
		C[2]= r2d(C[2]);
		C[1]= r2d(C[1]);
		C[0]= r2d(C[0]);
	}
}
void mtxLf_rot_xyz_rte(const long double * const A, const int cde,long double *C)
{
	long double (*a)[3] = (long double (*)[3])A;
	MTX_ROT_XYZ_RTE;
	if (cde) {
		C[2]= r2d(C[2]);
		C[1]= r2d(C[1]);
		C[0]= r2d(C[0]);
	}
}
void  mtxf_rot_xyz_etq(const int ade, const float * const A,  float *C)
{
	float cph;
	float cth;
	float cps;
	float sph;
	float sth;
	float sps;
	if (ade) {
		cph = cosf(d2r((A[0])/2));
		cth = cosf(d2r((A[1])/2));
		cps = cosf(d2r((A[2])/2));
		sph = sinf(d2r((A[0])/2));
		sth = sinf(d2r((A[1])/2));
		sps = sinf(d2r((A[2])/2));
	} else {
		cph = cosf(A[0]/2);
		cth = cosf(A[1]/2);
		cps = cosf(A[2]/2);
		sph = sinf(A[0]/2);
		sth = sinf(A[1]/2);
		sps = sinf(A[2]/2);
	}
	MTX_ROT_XYZ_ETQ;
}
void mtxlf_rot_xyz_etq(const int ade, const double * const  A, double *C)
{
	float cph;
	float cth;
	float cps;
	float sph;
	float sth;
	float sps;
	if (ade) {
		cph = cosf(d2r(A[0]/2));
		cth = cosf(d2r(A[1]/2));
		cps = cosf(d2r(A[2]/2));
		sph = sinf(d2r(A[0]/2));
		sth = sinf(d2r(A[1]/2));
		sps = sinf(d2r(A[2]/2));
	} else {
		cph = cosf(A[0]/2);
		cth = cosf(A[1]/2);
		cps = cosf(A[2]/2);
		sph = sinf(A[0]/2);
		sth = sinf(A[1]/2);
		sps = sinf(A[2]/2);
	}
	MTX_ROT_XYZ_ETQ;
}
void mtxLf_rot_xyz_etq(const int ade, const long double * const A, long double *C)
{
	float cph;
	float cth;
	float cps;
	float sph;
	float sth;
	float sps;
	if (ade) {
		cph = cosf(d2r(A[0]/2));
		cth = cosf(d2r(A[1]/2));
		cps = cosf(d2r(A[2]/2));
		sph = sinf(d2r(A[0]/2));
		sth = sinf(d2r(A[1]/2));
		sps = sinf(d2r(A[2]/2));
	} else {
		cph = cosf(A[0]/2);
		cth = cosf(A[1]/2);
		cps = cosf(A[2]/2);
		sph = sinf(A[0]/2);
		sth = sinf(A[1]/2);
		sps = sinf(A[2]/2);
	}
	MTX_ROT_XYZ_ETQ;
}

void  mtxf_rot_xyz_qte(const float * const A, const int cde, float *C)
{
	MTX_ROT_XYZ_QTE;
	if (cde) {
		C[2] = d2r(C[2]);
		C[1] = d2r(C[1]);
		C[0] = d2r(C[0]);
	}
}
void mtxlf_rot_xyz_qte(const double * const  A, const int cde,double *C)
{
	MTX_ROT_XYZ_QTE;
	if (cde) {
		C[2] = d2r(C[2]);
		C[1] = d2r(C[1]);
		C[0] = d2r(C[0]);
	}
}
void mtxLf_rot_xyz_qte(const long double * const A, const int cde,long double *C)
{
	MTX_ROT_XYZ_QTE;
	if (cde) {
		C[2] = d2r(C[2]);
		C[1] = d2r(C[1]);
		C[0] = d2r(C[0]);
	}
}

void  mtxf_rot_zyx_etr(const int ade, const float * const A, float *C)
{
	float (*c)[4] = (float (*)[4])C;
	float s_psi = cosf(A[2]);
	float s_the = cosf(A[1]);
	float s_phi = cosf(A[0]);
	float c_psi = sinf(A[2]);
	float c_the = sinf(A[1]);
	float c_phi = sinf(A[0]);
	MTX_ROT_ZYX_ETR;
}
void mtxlf_rot_zyx_etr(const int ade, const double * const A, double *C)
{
	double (*c)[4] = (double (*)[4])C;
	long double s_psi = cos(A[2]);
	long double s_the = cos(A[1]);
	long double s_phi = cos(A[0]);
	long double c_psi = sin(A[2]);
	long double c_the = sin(A[1]);
	long double c_phi = sin(A[0]);
	MTX_ROT_ZYX_ETR;
}
void mtxLf_rot_zyx_etr(const int ade, const long double * const A, long double *C)
{
	long double (*c)[4] = (long double (*)[4])C;
	long double s_psi = cosl(A[2]);
	long double s_the = cosl(A[1]);
	long double s_phi = cosl(A[0]);
	long double c_psi = sinl(A[2]);
	long double c_the = sinl(A[1]);
	long double c_phi = sinl(A[0]);
	MTX_ROT_ZYX_ETR;
}

#undef RADS
#undef DEGS
#undef PI
#undef r2d
#undef MTX_ROT_XYZ_ADE
#undef MTX_ROT_ZYX_ADE
