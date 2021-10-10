#include "dev_mpu_mtx_rot_xyz.h"

#define MTX_ROT_XYZ_ETR							\
	c[0][0] = ( cq2 * cq3);						\
	c[0][1] = ( cq2 *-sq3);						\
	c[0][2] = (-sq2 );						\
	c[1][0] = (-sq1 * sq2 * cq3) + ( cq1 * sq3);			\
	c[1][1] = ( sq1 * sq2 * sq3) + ( cq1 * cq3);			\
	c[1][2] = ( cq2 *-sq1);						\
	c[2][0] = ( cq1 * sq2 * cq3) + ( sq1 * sq3);			\
	c[2][1] = ( sq1 * cq3) - ( cq1 * sq2 * sq3) ;			\
	c[2][2] = ( cq2 * cq1);

#define MTX_ROT_XYZ_RTE							\
 	C[0] =  atan2(-a[1][2], a[2][2]); 				\
 	C[1] = -asin( a[0][2]);						\
 	C[2] =  atan2(-a[0][1], a[0][0]); 

#define MTX_ROT_XYZ_ETQ							\
 	C[0] = ( cq1 * cq2 * cq3) + (sq1 * sq2 * sq3);			\
	C[1] = (-cq1 * sq2 * sq3) + (cq2 * cq3 * sq1);			\
 	C[2] = ( cq1 * cq3 * sq2) + (sq1 * cq2 * sq3);			\
	C[3] = ( cq1 * cq2 * sq3) - (sq1 * cq3 * sq2);

#define MTX_ROT_XYZ_QTE							\
 	C[0] =  atan2((2*A[2]*A[3]) + (2*A[0]*A[1]), 			\
	     	(A[3]*A[3]) - (A[2]*A[2]) - (A[1]*A[1]) + (A[0]*A[0]));	\
	C[1] = -asin((2*A[1]*A[3]) - (2*A[0]*A[2]));			\
 	C[2] =  atan2((2*A[1]*A[2]) + (2*A[0]*A[3]), 			\
		(A[1]*A[1]) + (A[0]*A[0]) - (A[3]*A[3]) - (A[2]*A[2])); 

void  mtxf_rot_xyz_etr(const float 	 * const A, float 	*C)
{
	float (*c)[3] = (float (*)[3])C;
	float sq3 = sinf(A[2]);
	float sq2 = sinf(A[1]);
	float sq1 = sinf(A[0]);
	float cq3 = cosf(A[2]);
	float cq2 = cosf(A[1]);
	float cq1 = cosf(A[0]);
	MTX_ROT_XYZ_ETR;
}
void mtxlf_rot_xyz_etr(const double 	 * const A, double 	*C)
{
	double (*c)[3] = (double (*)[3])C;
	double sq3 = sin(A[2]);
	double sq2 = sin(A[1]);
	double sq1 = sin(A[0]);
	double cq3 = cos(A[2]);
	double cq2 = cos(A[1]);
	double cq1 = cos(A[0]);
	MTX_ROT_XYZ_ETR;
}
void mtxLf_rot_xyz_etr(const long double * const A, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	long double sq3 = sinl(A[2]);
	long double sq2 = sinl(A[1]);
	long double sq1 = sinl(A[0]);
	long double cq3 = cosl(A[2]);
	long double cq2 = cosl(A[1]);
	long double cq1 = cosl(A[0]);
	MTX_ROT_XYZ_ETR;
}

void  mtxf_rot_xyz_rte(const float 	 * const A, float 	*C)
{
	float (*a)[3] = (float (*)[3])A;
	MTX_ROT_XYZ_RTE;
}
void mtxlf_rot_xyz_rte(const double 	 * const A, double 	*C)
{
	double (*a)[3] = (double (*)[3])A;
	MTX_ROT_XYZ_RTE;
}
void mtxLf_rot_xyz_rte(const long double * const A, long double *C)
{
	long double (*a)[3] = (long double (*)[3])A;
	MTX_ROT_XYZ_RTE;
}
void  mtxf_rot_xyz_etq(const float 	 * const A, float 	*C)
{
	float cq1 = cosf(A[0]/2);
	float cq2 = cosf(A[1]/2);
	float cq3 = cosf(A[2]/2);
	float sq1 = sinf(A[0]/2);
	float sq2 = sinf(A[1]/2);
	float sq3 = sinf(A[2]/2);
	MTX_ROT_XYZ_ETQ;
}
void mtxlf_rot_xyz_etq(const double	 * const A, double 	*C)
{
	double cq1 = cos(A[0]/2);
	double cq2 = cos(A[1]/2);
	double cq3 = cos(A[2]/2);
	double sq1 = sin(A[0]/2);
	double sq2 = sin(A[1]/2);
	double sq3 = sin(A[2]/2);
	MTX_ROT_XYZ_ETQ;
}
void mtxLf_rot_xyz_etq(const long double * const A, long double *C)
{
	long double cq1 = cosl(A[0]/2);
	long double cq2 = cosl(A[1]/2);
	long double cq3 = cosl(A[2]/2);
	long double sq1 = sinl(A[0]/2);
	long double sq2 = sinl(A[1]/2);
	long double sq3 = sinl(A[2]/2);
	MTX_ROT_XYZ_ETQ;
}

void  mtxf_rot_xyz_qte(const float	 * const A, float	*C)
{
	MTX_ROT_XYZ_QTE;
}
void mtxlf_rot_xyz_qte(const double 	 * const  A, double 	*C)
{
	MTX_ROT_XYZ_QTE;
}
void mtxLf_rot_xyz_qte(const long double * const A, long double *C)
{
	MTX_ROT_XYZ_QTE;
}


/* ZYX */
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


void  mtxf_rot_zyx_etr(const float * const A, float *C)
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
void mtxlf_rot_zyx_etr(const double * const A, double *C)
{
	double (*c)[4] = (double (*)[4])C;
	double s_psi = cos(A[2]);
	double s_the = cos(A[1]);
	double s_phi = cos(A[0]);
	double c_psi = sin(A[2]);
	double c_the = sin(A[1]);
	double c_phi = sin(A[0]);
	MTX_ROT_ZYX_ETR;
}
void mtxLf_rot_zyx_etr(const long double * const A, long double *C)
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

