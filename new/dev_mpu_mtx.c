#include "dev_mpu_mtx.h"
#include <math.h>
#include <tgmath.h>

#define MTX_ZER								\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = 0;					\
		}							\
	}

#define MTX_EYE								\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			if (i == j)					\
				c[i][j] = 1;				\
			else						\
				c[i][j] = 0;				\
		}							\
	}

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
#define MTX_ROT_XYZ_RTE							\
 	C[0] =   atan2(a[1][2], a[2][2]); 				\
 	C[1] =  -asin( a[0][2]);					\
 	C[2] =   atan2(a[0][1], a[0][0]); 


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

#define MTX_CPY								\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = a[i][j];				\
		}							\
	}

#define MTX_TRN								\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = a[j][i];				\
		}							\
	}

#define MTX_ADD								\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = a[i][j] + b[i][j];			\
		}							\
	}

#define MTX_SUB								\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = a[i][j] - b[i][j];			\
		}							\
	}

#define MTX_MUL								\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			for (int k = 0; k < p; k++) {			\
				c[i][j] += (a[i][k] * b[k][j]);		\
			}						\
		}							\
	}
/* Invert a matrix usign the Gauss-Jordan method */
#define MTX_INV									\
int i, j;									\
for (i = 0; i < n; i++) {							\
	for (j = 0; j < n; j++) {						\
		c[i][j] = 0;							\
		b[i][j] = a[i][j];						\
	}									\
	c[i][i] = 1;								\
}										\
int iPass, imx, icol, irow;							\
for (iPass = 0; iPass < n; iPass++) {						\
	imx = iPass;								\
	for (irow = iPass; irow < n; irow++) {					\
		if (fabs(b[irow][iPass]) > fabs(b[imx][iPass])) {		\
			imx = irow;						\
		}								\
	}									\
	if (imx != iPass) {							\
		for (icol = 0; icol < n; icol++) {				\
			temp = c[iPass][icol];					\
			c[iPass][icol] = c[imx][icol];				\
			c[imx][icol] = temp;					\
			if (icol >= iPass) {					\
				temp = b[iPass][icol];				\
				b[iPass][icol] = b[imx][icol];			\
				b[imx][icol] = temp;				\
			}							\
		}								\
	}									\
	pivot = b[iPass][iPass];						\
	det  *= b[iPass][iPass];						\
	if (det == 0) {								\
		return -1;							\
	}									\
	for (icol = 0; icol < n; icol++) {					\
		c[iPass][icol] /=  pivot;					\
		if (icol >= iPass) {						\
			b[iPass][icol] /= pivot;				\
		}								\
	}									\
	for (irow = 0; irow < n; irow++) {					\
		if (irow != iPass) {						\
			factor = b[irow][iPass];				\
		}								\
		for (icol = 0; icol < n; icol++){				\
			if (irow != iPass){					\
				c[irow][icol] -= factor * c[iPass][icol];	\
				b[irow][icol] -= factor * b[iPass][icol];	\
			}							\
		}								\
	}									\
}										\
return 1;

#define MTX_ADD_SCL							\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = a[i][j] + k;					\
		}							\
	}

#define MTX_SUB_SCL							\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = a[i][j] - k;					\
		}							\
	}

#define MTX_MUL_SCL							\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = a[i][j] * k;					\
		}							\
	}

#define MTX_DIV_SCL							\
	for (int i = 0; i < m; i++) {					\
		for (int j = 0; j < n; j++) {				\
			c[i][j] = a[i][j] / k;					\
		}							\
	}

#define MTX_ROW_SWP			\
	for (int j = 0; j < n; j++) {	\
		tmp = a[k][j];		\
		a[k][j] = a[l][j];	\
		a[l][j] = tmp;		\
	}

#define MTX_ROW_ADD			\
	for (int j = 0; j < n; j++) {	\
		a[l][j] += a[k][j];	\
	}

#define MTX_ROW_ADM			\
	for (int j = 0; j < n; j++) {	\
		a[l][j] += (x*a[k][j]);	\
	}

#define MTX_ROW_SUB			\
	for (int j = 0; j < n; j++) {	\
		a[l][j] -= a[k][j];	\
	}

#define MTX_ROW_ADD_SCL			\
	for (int j = 0; j < n; j++) {	\
		a[l][j] += k;		\
	}

#define MTX_ROW_SUB_SCL			\
	for (int j = 0; j < n; j++) {	\
		a[l][j] -= k;		\
	}

#define MTX_ROW_MUL_SCL			\
	for (int j = 0; j < n; j++) {	\
		a[l][j] *= k;		\
	}

#define MTX_ROW_DIV_SCL			\
	for (int j = 0; j < n; j++) {	\
		a[l][j] /= k;		\
	}

#define MTX_COL_SWP			\
	for (int i = 0; i < m; i++) { 	\
		tmp = a[i][k];		\
		a[i][k] = a[i][l];	\
		a[i][l] = tmp;		\
	}

#define MTX_COL_ADD			\
	for (int i = 0; i < m; i++) {	\
		a[i][l] += a[i][k];	\
	}

#define MTX_COL_ADM			\
	for (int i = 0; i < m; i++) {	\
		a[i][l] += (x*a[i][k]);	\
	}

#define MTX_COL_SUB			\
	for (int i = 0; i < m; i++) {	\
		a[i][l] -= a[i][k];	\
	}

#define MTX_COL_ADD_SCL			\
	for (int i = 0; i < m; i++) {	\
		a[i][l] += k;		\
	}

#define MTX_COL_SUB_SCL			\
	for (int i = 0; i < m; i++) {	\
		a[i][l] -= k;		\
	}

#define MTX_COL_MUL_SCL			\
	for (int i = 0; i < m; i++) {	\
		a[i][l] *= k;		\
	}

#define MTX_COL_DIV_SCL			\
	for (int i = 0; i < m; i++) {	\
		a[i][l] /= k;		\
	}

/* C(mxn)=I will be changed to the zero matrix */
void mtxi_zer(const int m, const int n, int *C)
{
	int (*c)[n] = (int (*)[n])C;
	MTX_ZER;
}
void mtxf_zer(const int m, const int n, float *C)
{
	float (*c)[n] = (float (*)[n])C;
	MTX_ZER;
}
void mtxlf_zer(const int m, const int n, double *C)
{
	double (*c)[n] = (double (*)[n])C;
	MTX_ZER;
}
void mtxLf_zer(const int m, const int n, long double *C)
{
	long double (*c)[n] = (long double (*)[n])C;
	MTX_ZER;
}

/* C(mxn)=I will be changed to the identity matrix */
void mtxi_eye(const int m, const int n, int *C)
{
	int (*c)[n] = (int (*)[n])C;
	MTX_EYE;
}
void mtxf_eye(const int m, const int n, float *C)
{
	float (*c)[n] = (float (*)[n])C;
	MTX_EYE;
}
void mtxlf_eye(const int m, const int n, double *C)
{
	double (*c)[n] = (double (*)[n])C;
	MTX_EYE;
}
void mtxLf_eye(const int m, const int n, long double *C)
{
	long double (*c)[n] = (long double (*)[n])C;
	MTX_EYE;
}

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



void mtxi_cpy(const int * const A, const int m, const int n, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*c)[n] = (int (*)[n])C;
	MTX_CPY;
}
void mtxf_cpy(const float * A, const int m, const int n, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*c)[n] = (float (*)[n])C;
	MTX_CPY;
}
void mtxlf_cpy(const double * A, const int m, const int n, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*c)[n] = (double (*)[n])C;
	MTX_CPY;
}
void mtxLf_cpy(const long double * A, const int m, const int n, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_CPY;
}

void mtxi_trn(const int * const A, const int m, const int n, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*c)[n] = (int (*)[n])C;
	MTX_TRN;
}
void mtxf_trn(const float * A, const int m, const int n, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*c)[n] = (float (*)[n])C;
	MTX_TRN;
}
void mtxlf_trn(const double * A, const int m, const int n, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*c)[n] = (double (*)[n])C;
	MTX_TRN;
}
void mtxLf_trn(const long double * A, const int m, const int n, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_TRN;
}
void mtxi_add(const int * const A, const int * const B, const int m, const int n, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*b)[n] = (int (*)[n])B;
	int (*c)[n] = (int (*)[n])C;
	MTX_ADD;
}
void mtxf_add(const float * A, const float * B, const int m, const int n, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*b)[n] = (float (*)[n])B;
	float (*c)[n] = (float (*)[n])C;
	MTX_ADD;
}
void mtxlf_add(const double * A, const double * B, const int m, const int n, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*b)[n] = (double (*)[n])B;
	double (*c)[n] = (double (*)[n])C;
	MTX_ADD;
}
void mtxLf_add(const long double * A, const long double * B, const int m, const int n, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*b)[n] = (long double (*)[n])B;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_ADD;
}
void mtxi_sub(const int * const A, const int * const B, const int m, const int n, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*b)[n] = (int (*)[n])B;
	int (*c)[n] = (int (*)[n])C;
	MTX_SUB;
}
void mtxf_sub(const float * A, const float * B, const int m, const int n, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*b)[n] = (float (*)[n])B;
	float (*c)[n] = (float (*)[n])C;
	MTX_SUB;
}
void mtxlf_sub(const double * A, const double * B, const int m, const int n, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*b)[n] = (double (*)[n])B;
	double (*c)[n] = (double (*)[n])C;
	MTX_SUB;
}
void mtxLf_sub(const long double * A, const long double * B, const int m, const int n, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*b)[n] = (long double (*)[n])B;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_SUB;
}

/* C(mxn) = A(mxp) * B(pxn) */
void mtxi_mul(const int * const A, const int * const B, const int m, const int p, const int n, int *C)
{
	int (*a)[p] = (int (*)[p])A;
	int (*b)[n] = (int (*)[n])B;
	int (*c)[n] = (int (*)[n])C;
	MTX_MUL;
}
void mtxf_mul(const float * A, const float * B, const int m, const int p, const int n, float *C)
{
	float (*a)[p] = (float (*)[p])A;
	float (*b)[n] = (float (*)[n])B;
	float (*c)[n] = (float (*)[n])C;
	MTX_MUL;
}
void mtxlf_mul(const double * A, const double * B, const int m, const int p, const int n, double *C)
{
	double (*a)[p] = (double (*)[p])A;
	double (*b)[n] = (double (*)[n])B;
	double (*c)[n] = (double (*)[n])C;
	MTX_MUL;
}
void mtxLf_mul(const long double * A, const long double * B, const int m, const int p, const int n, long double *C)
{
	long double (*a)[p] = (long double (*)[p])A;
	long double (*b)[n] = (long double (*)[n])B;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_MUL;
}

/* C(mxn) = A(mxn) + k*J(mxn)	Multiply each element of A(mxn) by constant k */
void  mtxi_add_scl(const int * const A,  const int m, const int n, int k, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*c)[n] = (int (*)[n])C;
	MTX_ADD_SCL;
}
void  mtxf_add_scl(const float * const A, const int m, const int n, float k, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*c)[n] = (float (*)[n])C;
	MTX_ADD_SCL;
}
void mtxlf_add_scl(const double * const A, const int m, const int n, double k, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*c)[n] = (double (*)[n])C;
	MTX_ADD_SCL;
}
void mtxLf_add_scl(const long double * const A, const int m, const int n, long double k, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_ADD_SCL;
}

/* C(mxn) = A(mxn) - k*J(mxn)	Subtract constant k from each element of A(mxn) */
void  mtxi_sub_scl(const int * const A,  const int m, const int n, int k, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*c)[n] = (int (*)[n])C;
	MTX_SUB_SCL;
}
void  mtxf_sub_scl(const float * const A, const int m, const int n, float k, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*c)[n] = (float (*)[n])C;
	MTX_SUB_SCL;
}
void mtxlf_sub_scl(const double * const A, const int m, const int n, double k, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*c)[n] = (double (*)[n])C;
	MTX_SUB_SCL;
}
void mtxLf_sub_scl(const long double * const A, const int m, const int n, long double k, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_SUB_SCL;
}

/* C(mxn) = A(mxp) * k*J(mxn)	Multiply each element of A(mxn) by constant k */
void  mtxi_mul_scl(const int * const A,  const int m, const int n, int k, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*c)[n] = (int (*)[n])C;
	MTX_MUL_SCL;
}
void  mtxf_mul_scl(const float * const A, const int m, const int n, float k, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*c)[n] = (float (*)[n])C;
	MTX_MUL_SCL;
}
void mtxlf_mul_scl(const double * const A, const int m, const int n, double k, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*c)[n] = (double (*)[n])C;
	MTX_MUL_SCL;
}
void mtxLf_mul_scl(const long double * const A, const int m, const int n, long double k, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_MUL_SCL;
}

/* C(mxn) = A(mxp) / k*J(mxn)	Divide each element of A(mxn) by constant k */
void  mtxi_div_scl(const int * const A,  const int m, const int n, int k, int *C)
{
	int (*a)[n] = (int (*)[n])A;
	int (*c)[n] = (int (*)[n])C;
	MTX_DIV_SCL;
}
void  mtxf_div_scl(const float * const A, const int m, const int n, float k, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*c)[n] = (float (*)[n])C;
	MTX_DIV_SCL;
}
void mtxlf_div_scl(const double * const A, const int m, const int n, double k, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*c)[n] = (double (*)[n])C;
	MTX_DIV_SCL;
}
void mtxLf_div_scl(const long double * const A, const int m, const int n, long double k, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*c)[n] = (long double (*)[n])C;
	MTX_DIV_SCL;
}

/* ROW OPERATIONS */
void mtxi_row_swp(int * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	int (*a)[n] = (int (*)[n])A;
	int tmp;
	MTX_ROW_SWP;
}
void mtxf_row_swp(float * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	float (*a)[n] = (float (*)[n])A;
	float tmp;
	MTX_ROW_SWP;
}
void mtxlf_row_swp(double * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	double (*a)[n] = (double (*)[n])A;
	double tmp;
	MTX_ROW_SWP;
}
void mtxLf_row_swp(long double * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	long double tmp;
	MTX_ROW_SWP;
}

void  mtxi_row_add(int * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	int (*a)[n] = (int (*)[n])A;
	MTX_ROW_ADD;
}
void  mtxf_row_add(float * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	float (*a)[n] = (float (*)[n])A;
	MTX_ROW_ADD;
}
void mtxlf_row_add(double * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	double (*a)[n] = (double (*)[n])A;
	MTX_ROW_ADD;
}
void mtxLf_row_add(long double * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	MTX_ROW_ADD;
}

void  mtxi_row_adm(int * A, const int n, int x, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	int (*a)[n] = (int (*)[n])A;
	MTX_ROW_ADM;
}
void  mtxf_row_adm(float * A, const int n, float x, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	float (*a)[n] = (float (*)[n])A;
	MTX_ROW_ADM;
}
void mtxlf_row_adm(double * A, const int n, double x, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	double (*a)[n] = (double (*)[n])A;
	MTX_ROW_ADM;
}
void mtxLf_row_adm(long double * A, const int n, long double x, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	MTX_ROW_ADM;
}

/*  L(n) += k	add scalar 'k' to row 'l' on matrix A(_xn) */
void  mtxi_row_add_scl(int * A, const int n, const int k, int l)
{
	l--; /* l index starts at 1 */
	int (*a)[n] = (int (*)[n])A;
	MTX_ROW_ADD_SCL;
}

void  mtxi_row_sub(int * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	int (*a)[n] = (int (*)[n])A;
	MTX_ROW_SUB;
}
void  mtxf_row_sub(float * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	float (*a)[n] = (float (*)[n])A;
	MTX_ROW_SUB;
}
void mtxlf_row_sub(double * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	double (*a)[n] = (double (*)[n])A;
	MTX_ROW_SUB;
}
void mtxLf_row_sub(long double * A, const int n, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	MTX_ROW_SUB;
}

void  mtxf_row_add_scl(float * A, const int n, const float k, int l)
{
	l--; /* l index starts at 1 */
	float (*a)[n] = (float (*)[n])A;
	MTX_ROW_ADD_SCL;
}
void mtxlf_row_add_scl(double * A, const int n, const double k, int l)
{
	l--; /* l index starts at 1 */
	double (*a)[n] = (double (*)[n])A;
	MTX_ROW_ADD_SCL;
}
void mtxLf_row_add_scl(long double * A, const int n, const long double k, int l)
{
	l--; /* l index starts at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	MTX_ROW_ADD_SCL;
}

/*  L(n) -= k	subtract scalar 'k' from row 'l' on matrix A(_xn) */
void  mtxi_row_sub_scl(int * A, const int n, const int k, int l)
{
	l--; /* l index starts at 1 */
	int (*a)[n] = (int (*)[n])A;
	MTX_ROW_SUB_SCL;
}
void  mtxf_row_sub_scl(float * A, const int n, const float k, int l)
{
	l--; /* l index starts at 1 */
	float (*a)[n] = (float (*)[n])A;
	MTX_ROW_SUB_SCL;
}
void mtxlf_row_sub_scl(double * A, const int n, const double k, int l)
{
	l--; /* l index starts at 1 */
	double (*a)[n] = (double (*)[n])A;
	MTX_ROW_SUB_SCL;
}
void mtxLf_row_sub_scl(long double * A, const int n, const long double k, int l)
{
	l--; /* l index starts at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	MTX_ROW_SUB_SCL;
}

/*  L(n) *= k	multiply row 'l' by scalar 'k' on matrix A(_xn) */
void  mtxi_row_mul_scl(int * A, const int n, const int k, int l)
{
	l--; /* l index starts at 1 */
	int (*a)[n] = (int (*)[n])A;
	MTX_ROW_MUL_SCL;
}
void  mtxf_row_mul_scl(float * A, const int n, const float k, int l)
{
	l--; /* l index starts at 1 */
	float (*a)[n] = (float (*)[n])A;
	MTX_ROW_MUL_SCL;
}
void mtxlf_row_mul_scl(double * A, const int n, const double k, int l)
{
	l--; /* l index starts at 1 */
	double (*a)[n] = (double (*)[n])A;
	MTX_ROW_MUL_SCL;
}
void mtxLf_row_mul_scl(long double * A, const int n, const long double k, int l)
{
	l--; /* l index starts at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	MTX_ROW_MUL_SCL;
}

/*  L(n) \= k	divide row 'l' by scalar 'k' on matrix A(_xn) */
void  mtxi_row_div_scl(int * A, const int n, const int k, int l)
{
	l--; /* l index starts at 1 */
	int (*a)[n] = (int (*)[n])A;
	MTX_ROW_DIV_SCL;
}
void  mtxf_row_div_scl(float * A, const int n, const float k, int l)
{
	l--; /* l index starts at 1 */
	float (*a)[n] = (float (*)[n])A;
	MTX_ROW_DIV_SCL;
}
void mtxlf_row_div_scl(double * A, const int n, const double k, int l)
{
	l--; /* l index starts at 1 */
	double (*a)[n] = (double (*)[n])A;
	MTX_ROW_DIV_SCL;
}
void mtxLf_row_div_scl(long double * A, const int n, const long double k, int l)
{
	l--; /* l index starts at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	MTX_ROW_DIV_SCL;
}


void mtxi_col_swp(int * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	int (*a)[m] = (int (*)[m])A;
	int tmp;
	MTX_COL_SWP;
}
void mtxf_col_swp(float * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	float (*a)[m] = (float (*)[m])A;
	float tmp;
	MTX_COL_SWP;
}
void mtxlf_col_swp(double * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	double (*a)[m] = (double (*)[m])A;
	double tmp;
	MTX_COL_SWP;
}
void mtxLf_col_swp(long double * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	long double tmp;
	MTX_COL_SWP;
}

/*  L(m) += K(m)		add column 'k' to column 'l' on matrix A(mx_) */
void  mtxi_col_add(int * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	int (*a)[m] = (int (*)[m])A;
	MTX_COL_ADD;
}
void  mtxf_col_add(float * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	float (*a)[m] = (float (*)[m])A;
	MTX_COL_ADD;
}
void mtxlf_col_add(double * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	double (*a)[m] = (double (*)[m])A;
	MTX_COL_ADD;
}
void mtxLf_col_add(long double * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	MTX_COL_ADD;
}

void  mtxi_col_adm(int * A, const int m, int x, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	int (*a)[m] = (int (*)[m])A;
	MTX_COL_ADM;
}
void  mtxf_col_adm(float * A, const int m, float x, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	float (*a)[m] = (float (*)[m])A;
	MTX_COL_ADM;
}
void mtxlf_col_adm(double * A, const int m, double x, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	double (*a)[m] = (double (*)[m])A;
	MTX_COL_ADM;
}
void mtxLf_col_adm(long double * A, const int m, long double x, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	MTX_COL_ADM;
}

/*  L(m) -= K(m)		subtract column 'k' from column 'l' on matrix A(mx_) */
void  mtxi_col_sub(int * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	int (*a)[m] = (int (*)[m])A;
	MTX_COL_SUB;
}
void  mtxf_col_sub(float * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	float (*a)[m] = (float (*)[m])A;
	MTX_COL_SUB;
}
void mtxlf_col_sub(double * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	double (*a)[m] = (double (*)[m])A;
	MTX_COL_SUB;
}
void mtxLf_col_sub(long double * A, const int m, int k, int l)
{
	k--; /* k index starts at 1 */
	l--; /* l index starts at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	MTX_COL_SUB;
}

/* L(m) += k	add scalar 'k' to col 'l' on matrix A(mx_) */
void  mtxi_col_add_scl(int * A, const int m, const int k, int l)
{
	l--; /* l index starts at 1 */
	int (*a)[m] = (int (*)[m])A;
	MTX_COL_ADD_SCL;
}
void  mtxf_col_add_scl(float * A, const int m, const float k, int l)
{
	l--; /* l index starts at 1 */
	float (*a)[m] = (float (*)[m])A;
	MTX_COL_ADD_SCL;
}
void mtxlf_col_add_scl(double * A, const int m, const double k, int l)
{
	l--; /* l index starts at 1 */
	double (*a)[m] = (double (*)[m])A;
	MTX_COL_ADD_SCL;
}
void mtxLf_col_add_scl(long double * A, const int m, const long double k, int l)
{
	l--; /* l index starts at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	MTX_COL_ADD_SCL;
}

/* L(m) -= k	subtract scalar 'k' from col 'l' on matrix A(mx_) */
void  mtxi_col_sub_scl(int * A, const int m, int k, int l)
{
	l--; /* l index starts at 1 */
	int (*a)[m] = (int (*)[m])A;
	MTX_COL_SUB_SCL;
}
void  mtxf_col_sub_scl(float * A, const int m, const float k, int l)
{
	float (*a)[m] = (float (*)[m])A;
	MTX_COL_SUB_SCL;
}
void mtxlf_col_sub_scl(double * A, const int m, const double k, int l)
{
	l--; /* l index starts at 1 */
	double (*a)[m] = (double (*)[m])A;
	MTX_COL_SUB_SCL;
}
void mtxLf_col_sub_scl(long double * A, const int m, const long double k, int l)
{
	l--; /* l index starts at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	MTX_COL_SUB_SCL;
}

/* L(m) *= k	multiply col 'l' by scalar 'k' on matrix A(mx_) */
void  mtxi_col_mul_scl(int * A, const int m, const int k, int l)
{
	l--; /* l index starts at 1 */
	int (*a)[m] = (int (*)[m])A;
	MTX_COL_MUL_SCL;
}
void  mtxf_col_mul_scl(float * A, const int m, const float k, int l)
{
	l--; /* l index starts at 1 */
	float (*a)[m] = (float (*)[m])A;
	MTX_COL_MUL_SCL;
}
void mtxlf_col_mul_scl(double * A, const int m, const double k, int l)
{
	l--; /* l index starts at 1 */
	double (*a)[m] = (double (*)[m])A;
	MTX_COL_MUL_SCL;
}
void mtxLf_col_mul_scl(long double * A, const int m, const long double k, int l)
{
	l--; /* l index starts at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	MTX_COL_MUL_SCL;
}

/* L(m) \= k	divide col 'l' by scalar 'k' on matrix A(mx_) */
void  mtxi_col_div_scl(int * A, const int m, const int k, int l)
{
	l--; /* l index starts at 1 */
	int (*a)[m] = (int (*)[m])A;
	MTX_COL_DIV_SCL;
}
void  mtxf_col_div_scl(float * A, const int m, const float k, int l)
{
	l--; /* l index starts at 1 */
	float (*a)[m] = (float (*)[m])A;
	MTX_COL_DIV_SCL;
}
void mtxlf_col_div_scl(double * A, const int m, const double k, int l)
{
	l--; /* l index starts at 1 */
	double (*a)[m] = (double (*)[m])A;
	MTX_COL_DIV_SCL;
}
void mtxLf_col_div_scl(long double * A, const int m, const long double k, int l)
{
	l--; /* l index starts at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	MTX_COL_DIV_SCL;
}

/*
 * Inverts a matrix using the Gauss Jordan method.
 * returns 0 on success, 1 on failure.
 */
#include <math.h>
int mtxf_inv(float * A, float * B, int n, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*b)[n] = (float (*)[n])B;
	float (*c)[n] = (float (*)[n])C;

	float det = 1;
	float temp, pivot, factor;
	MTX_INV;
}
int mtxlf_inv(double * A, double * B, int n, double *C)
{
	double (*a)[n] = (double (*)[n])A;
	double (*b)[n] = (double (*)[n])B;
	double (*c)[n] = (double (*)[n])C;

	double det = 1;
	double temp, pivot, factor;
	MTX_INV;
}
int mtxLf_inv(long double * A, long double * B, int n, long double *C)
{
	long double (*a)[n] = (long double (*)[n])A;
	long double (*b)[n] = (long double (*)[n])B;
	long double (*c)[n] = (long double (*)[n])C;

	long double det = 1;
	long double temp, pivot, factor;
	MTX_INV;
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
/* unit converion in extra precision */
#define PI (3.1415926535897932384L)
#define r2d(q) ((q)*(180.0L/PI)) 
#define d2r(q) ((q)*(PI/180.0L)) 
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

void mtxi_pri(const int * const A, const int m, const int n)
{
	int (*a)[n] = (int (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			printf("%d ", a[i][j]);
		}
		printf("\n");
	}
}
void mtxf_pri(const float * const A, const int m, const int n)
{
	float (*a)[n] = (float (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			printf("%f ", a[i][j]);
		}
		printf("\n");
	}
}
void mtxlf_pri(double *A, const int m, const int n)
{
	double (*a)[n] = (double (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			printf("%lf ", a[i][j]);
		}
		printf("\n");
	}
}
void mtxLf_pri(long double *A, const int m, const int n)
{
	long double (*a)[n] = (long double (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			printf("%Lf ", a[i][j]);
		}
		printf("\n");
	}
}
void mtxi_pri_row(const int * const A, const int n, int k)
{
	k--; /* indexes start at 1 */
	int (*a)[n] = (int (*)[n])A;
	for (int j = 0; j < n; j++) {
		printf("%d ", a[k][j]);
	}
	printf("\n");
}
void mtxf_pri_row(const float * const A, const int n, int k)
{
	k--; /* indexes start at 1 */
	float (*a)[n] = (float (*)[n])A;
	for (int j = 0; j < n; j++) {
		printf("%f ", a[k][j]);
	}
	printf("\n");
}
void mtxlf_pri_row(const double * const A, const int n, int k)
{
	k--; /* indexes start at 1 */
	double (*a)[n] = (double (*)[n])A;
	for (int j = 0; j < n; j++) {
		printf("%lf ", a[k][j]);
	}
	printf("\n");
}
void mtxLf_pri_row(const long double * const A, const int n, int k)
{
	k--; /* indexes start at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	for (int j = 0; j < n; j++) {
		printf("%Lf ", a[k][j]);
	}
	printf("\n");
}
void mtxi_pri_col(const int * const A, const int m, int k)
{
	k--; /* indexes start at 1 */
	int (*a)[m] = (int (*)[m])A;
	for (int i = 0; i < m; i++) {
		printf("%d\n", a[i][k]);
	}
}
void mtxf_pri_col(const float * const A, const int m, int k)
{
	k--; /* indexes start at 1 */
	float (*a)[m] = (float (*)[m])A;
	for (int i = 0; i < m; i++) {
		printf("%f\n", a[i][k]);
	}
}
void mtxlf_pri_col(const double * const A, const int m, int k)
{
	k--; /* indexes start at 1 */
	double (*a)[m] = (double (*)[m])A;
	for (int i = 0; i < m; i++) {
		printf("%lf\n", a[i][k]);
	}
}
void mtxLf_pri_col(const long double * const A, const int m, int k)
{
	k--; /* indexes start at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	for (int i = 0; i < m; i++) {
		printf("%Lf\n", a[i][k]);
	}
}
void mtxi_spr(const int * const A, const int m, const int n, char *s)
{
	int (*a)[n] = (int (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			sprintf(s, "%d ", a[i][j]);
		}
		sprintf(s, "\n");
	}
}
void mtxf_spr(const float * const A, const int m, const int n, char *s)
{
	float (*a)[n] = (float (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			sprintf(s, "%f ", a[i][j]);
		}
		sprintf(s, "\n");
	}
}
void mtxlf_spr(double *A, const int m, const int n, char *s)
{
	double (*a)[n] = (double (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			sprintf(s, "%lf ", a[i][j]);
		}
		sprintf(s, "\n");
	}
}
void mtxLf_spr(long double *A, const int m, const int n, char *s)
{
	long double (*a)[n] = (long double (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			sprintf(s, "%Lf ", a[i][j]);
		}
		sprintf(s, "\n");
	}
}
void mtxi_spr_row(const int * const A, const int n, int k, char *s)
{
	k--; /* indexes start at 1 */
	int (*a)[n] = (int (*)[n])A;
	for (int j = 0; j < n; j++) {
		sprintf(s, "%d ", a[k][j]);
	}
	sprintf(s, "\n");
}
void mtxf_spr_row(const float * const A, const int n, int k, char *s)
{
	k--; /* indexes start at 1 */
	float (*a)[n] = (float (*)[n])A;
	for (int j = 0; j < n; j++) {
		sprintf(s, "%f ", a[k][j]);
	}
	sprintf(s, "\n");
}
void mtxlf_spr_row(const double * const A, const int n, int k, char *s)
{
	k--; /* indexes start at 1 */
	double (*a)[n] = (double (*)[n])A;
	for (int j = 0; j < n; j++) {
		sprintf(s, "%lf ", a[k][j]);
	}
	sprintf(s, "\n");
}
void mtxLf_spr_row(const long double * const A, const int n, int k, char *s)
{
	k--; /* indexes start at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	for (int j = 0; j < n; j++) {
		sprintf(s, "%Lf ", a[k][j]);
	}
	sprintf(s, "\n");
}
void mtxi_spr_col(const int * const A, const int m, int k, char *s)
{
	k--; /* indexes start at 1 */
	int (*a)[m] = (int (*)[m])A;
	for (int i = 0; i < m; i++) {
		sprintf(s, "%d\n", a[i][k]);
	}
}
void mtxf_spr_col(const float * const A, const int m, int k, char *s)
{
	k--; /* indexes start at 1 */
	float (*a)[m] = (float (*)[m])A;
	for (int i = 0; i < m; i++) {
		sprintf(s, "%f\n", a[i][k]);
	}
}
void mtxlf_spr_col(const double * const A, const int m, int k, char *s)
{
	k--; /* indexes start at 1 */
	double (*a)[m] = (double (*)[m])A;
	for (int i = 0; i < m; i++) {
		sprintf(s, "%lf\n", a[i][k]);
	}
}
void mtxLf_spr_col(const long double * const A, const int m, int k, char *s)
{
	k--; /* indexes start at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	for (int i = 0; i < m; i++) {
		sprintf(s, "%Lf\n", a[i][k]);
	}
}

void mtxi_fpr(const int * const A, const int m, const int n, FILE *f)
{
	int (*a)[n] = (int (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			fprintf(f, "%d ", a[i][j]);
		}
		fprintf(f, "\n");
	}
}
void mtxf_fpr(const float * const A, const int m, const int n, FILE *f)
{
	float (*a)[n] = (float (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			fprintf(f, "%f ", a[i][j]);
		}
		fprintf(f, "\n");
	}
}
void mtxlf_fpr(double *A, const int m, const int n, FILE *f)
{
	double (*a)[n] = (double (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			fprintf(f, "%lf ", a[i][j]);
		}
		fprintf(f, "\n");
	}
}
void mtxLf_fpr(long double *A, const int m, const int n, FILE *f)
{
	long double (*a)[n] = (long double (*)[n])A;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			fprintf(f, "%Lf ", a[i][j]);
		}
		fprintf(f, "\n");
	}
}
void mtxi_fpr_row(const int * const A, const int n, int k, FILE *f)
{
	k--; /* indexes start at 1 */
	int (*a)[n] = (int (*)[n])A;
	for (int j = 0; j < n; j++) {
		fprintf(f, "%d ", a[k][j]);
	}
	fprintf(f, "\n");
}
void mtxf_fpr_row(const float * const A, const int n, int k, FILE *f)
{
	k--; /* indexes start at 1 */
	float (*a)[n] = (float (*)[n])A;
	for (int j = 0; j < n; j++) {
		fprintf(f, "%f ", a[k][j]);
	}
	fprintf(f, "\n");
}
void mtxlf_fpr_row(const double * const A, const int n, int k, FILE *f)
{
	k--; /* indexes start at 1 */
	double (*a)[n] = (double (*)[n])A;
	for (int j = 0; j < n; j++) {
		fprintf(f, "%lf ", a[k][j]);
	}
	fprintf(f, "\n");
}
void mtxLf_fpr_row(const long double * const A, const int n, int k, FILE *f)
{
	k--; /* indexes start at 1 */
	long double (*a)[n] = (long double (*)[n])A;
	for (int j = 0; j < n; j++) {
		fprintf(f, "%Lf ", a[k][j]);
	}
	fprintf(f, "\n");
}
void mtxi_fpr_col(const int * const A, const int m, int k, FILE *f)
{
	k--; /* indexes start at 1 */
	int (*a)[m] = (int (*)[m])A;
	for (int i = 0; i < m; i++) {
		fprintf(f, "%d\n", a[i][k]);
	}
}
void mtxf_fpr_col(const float * const A, const int m, int k, FILE *f)
{
	k--; /* indexes start at 1 */
	float (*a)[m] = (float (*)[m])A;
	for (int i = 0; i < m; i++) {
		fprintf(f, "%f\n", a[i][k]);
	}
}
void mtxlf_fpr_col(const double * const A, const int m, int k, FILE *f)
{
	k--; /* indexes start at 1 */
	double (*a)[m] = (double (*)[m])A;
	for (int i = 0; i < m; i++) {
		fprintf(f, "%lf\n", a[i][k]);
	}
}
void mtxLf_fpr_col(const long double * const A, const int m, int k, FILE *f)
{
	k--; /* indexes start at 1 */
	long double (*a)[m] = (long double (*)[m])A;
	for (int i = 0; i < m; i++) {
		fprintf(f, "%Lf\n", a[i][k]);
	}
}
