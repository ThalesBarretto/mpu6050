#include "dev_mpu_mtx_rot.h"

/* Use to  rotate a vector by q(rh) around X 	*/
#define MTX_ROT_RVX 				\
	c[0][0] =	1;			\
	c[0][1] =	0;			\
	c[0][2] =	0;			\
	c[1][0] =	0;			\
	c[1][1] =	cq;			\
	c[1][2] =	-sq;			\
	c[2][0] =	0;			\
	c[2][1] =	sq;			\
	c[2][2] =	cq;

/* Use to rotate a vector by q(rh) around Y 	*/
#define MTX_ROT_RVY 				\
	c[0][0] =	cq;			\
	c[0][1] =	0;			\
	c[0][2] =	sq;			\
	c[1][0] =	0;			\
	c[1][1] =	1;			\
	c[1][2] =	0;			\
	c[2][0] =	-sq;			\
	c[2][1] =	0;			\
	c[2][2] =	cq;

/* Use to rotate a vector by q(rh) around Z 	*/
#define MTX_ROT_RVZ 				\
	c[0][0] =	cq;			\
	c[0][1] =	-sq;			\
	c[0][2] =	0;			\
	c[1][0] =	sq;			\
	c[1][1] =	cq;			\
	c[1][2] =	0;			\
	c[2][0] =	0;			\
	c[2][1] =	0;			\
	c[2][2] =	1;

/* Use to  rotate a frame by q(rh) around X 	*/
#define MTX_ROT_RFX 				\
	c[0][0] =	1;			\
	c[0][1] =	0;			\
	c[0][2] =	0;			\
	c[1][0] =	0;			\
	c[1][1] =	cq;			\
	c[1][2] =	sq;			\
	c[2][0] =	0;			\
	c[2][1] =	-sq;			\
	c[2][2] =	cq;

/* Use to rotate a frame by q(rh) around Y 	*/
#define MTX_ROT_RFY 				\
	c[0][0] =	cq;			\
	c[0][1] =	0;			\
	c[0][2] =	-sq;			\
	c[1][0] =	0;			\
	c[1][1] =	1;			\
	c[1][2] =	0;			\
	c[2][0] =	sq;			\
	c[2][1] =	0;			\
	c[2][2] =	cq;

/* Use to rotate a frame by q(rh) around Z 	*/
#define MTX_ROT_RFZ 				\
	c[0][0] =	cq;			\
	c[0][1] =	sq;			\
	c[0][2] =	0;			\
	c[1][0] =	-sq;			\
	c[1][1] =	cq;			\
	c[1][2] =	0;			\
	c[2][0] =	0;			\
	c[2][1] =	0;			\
	c[2][2] =	1;
/* CPM - Cross product matrix 	*/
#define MTX_ROT_CPM		\
	c[0][0] =  0;		\
	c[0][1] = -A[2];	\
	c[0][2] =  A[1];	\
	c[1][0] =  A[2];	\
	c[1][1] =  0;		\
	c[1][2] = -A[0];	\
	c[2][0] = -A[1];	\
	c[2][1] =  A[0];	\
	c[2][2] =  0;

/* CP2 - Cross product matrix squared 			*/
#define MTX_ROT_CP2					\
	c[0][0] = -(A[2] * A[2]) -(A[1] * A[1])  ;	\
	c[0][1] =  (A[1] * A[0] );			\
	c[0][2] =  (A[2] * A[0]) ;			\
	c[1][0] =  (A[1] * A[0]) ;			\
	c[1][1] = -(A[2] * A[2]) -(A[0] * A[0])  ;	\
	c[1][2] =  (A[2] * A[1]) ;			\
	c[2][0] =  (A[2] * A[0]) ;			\
	c[2][1] =  (A[2] * A[1]) ;			\
	c[2][2] = -(A[1] * A[1]) -(A[0] * A[0]) ;


void  mtxf_rot_rvx(const float q, float *C)
{
	float (*c)[3] = (float (*)[3])C;
	float cq = cosf(q);
	float sq = sinf(q);
	MTX_ROT_RVX;
}
void mtxlf_rot_rvx(const double q, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	double cq = cos(q);
	double sq = sin(q);
	MTX_ROT_RVX;
}
void mtxLf_rot_rvx(const long double q, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	long double cq = cosl(q);
	long double sq = sinl(q);
	MTX_ROT_RVX;
}
void  mtxf_rot_rvy(const float q, float *C)
{
	float (*c)[3] = (float (*)[3])C;
	float cq = cosf(q);
	float sq = sinf(q);
	MTX_ROT_RVY;
}
void mtxlf_rot_rvy(const double q, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	double cq = cos(q);
	double sq = sin(q);
	MTX_ROT_RVY;
}
void mtxLf_rot_rvy(const long double q, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	long double cq = cosl(q);
	long double sq = sinl(q);
	MTX_ROT_RVY;
}
void  mtxf_rot_rvz(const float q, float *C)
{
	float (*c)[3] = (float (*)[3])C;
	float cq = cosf(q);
	float sq = sinf(q);
	MTX_ROT_RVZ;
}
void mtxlf_rot_rvz(const double q, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	double cq = cos(q);
	double sq = sin(q);
	MTX_ROT_RVZ;
}
void mtxLf_rot_rvz(const long double q, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	long double cq = cosl(q);
	long double sq = sinl(q);
	MTX_ROT_RVZ;
}

void  mtxf_rot_rfx(const float q, float *C)
{
	float (*c)[3] = (float (*)[3])C;
	float cq = cosf(q);
	float sq = sinf(q);
	MTX_ROT_RFX;
}
void mtxlf_rot_rfx(const double q, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	double cq = cos(q);
	double sq = sin(q);
	MTX_ROT_RFX;
}
void mtxLf_rot_rfx(const long double q, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	long double cq = cosl(q);
	long double sq = sinl(q);
	MTX_ROT_RFX;
}
void  mtxf_rot_rfy(const float q, float *C)
{
	float (*c)[3] = (float (*)[3])C;
	float cq = cosf(q);
	float sq = sinf(q);
	MTX_ROT_RFY;
}
void mtxlf_rot_rfy(const double q, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	double cq = cos(q);
	double sq = sin(q);
	MTX_ROT_RFY;
}
void mtxLf_rot_rfy(const long double q, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	long double cq = cosl(q);
	long double sq = sinl(q);
	MTX_ROT_RFY;
}
void  mtxf_rot_rfz(const float q, float *C)
{
	float (*c)[3] = (float (*)[3])C;
	float cq = cosf(q);
	float sq = sinf(q);
	MTX_ROT_RFZ;
}
void mtxlf_rot_rfz(const double q, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	double cq = cos(q);
	double sq = sin(q);
	MTX_ROT_RFZ;
}
void mtxLf_rot_rfz(const long double q, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	long double cq = cosl(q);
	long double sq = sinl(q);
	MTX_ROT_RFZ;
}
void  mtxf_rot_cpm(const float * const A,  float *C)
{
	float (*c)[3] = (float (*)[3])C;
	MTX_ROT_CPM;
}
void mtxlf_rot_cpm(const double * const  A, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	MTX_ROT_CPM;
}
void mtxLf_rot_cpm(const long double * const A, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	MTX_ROT_CPM;
}
void  mtxf_rot_cp2(const float * const A,  float *C)
{
	float (*c)[3] = (float (*)[3])C;
	MTX_ROT_CP2;
}
void mtxlf_rot_cp2(const double * const  A, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	MTX_ROT_CP2;
}
void mtxLf_rot_cp2(const long double * const A, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	MTX_ROT_CP2;
}

