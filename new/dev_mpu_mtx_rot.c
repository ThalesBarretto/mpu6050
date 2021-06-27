#include "dev_mpu_mtx_rot.h"
#define MTX_ROX \
	c[0][0] =	1;\
	c[0][1] =	0;\
	c[0][2] =	0;\
	c[0][3] =	0;\
	c[1][0] =	0;\
	c[1][1] =	cq;\
	c[1][2] =	sq;\
	c[1][3] =	0;\
	c[2][0] =	0;\
	c[2][1] =	-sq;\
	c[2][2] =	cq;\
	c[2][3] =	0;\
	c[3][0] =	0;\
	c[3][1] =	0;\
	c[3][2] =	0;\
	c[3][3] =	1;\

#define MTX_ROY \
	c[0][0] =	cq;\
	c[0][1] =	0;\
	c[0][2] =	sq;\
	c[0][3] =	0;\
	c[1][0] =	0;\
	c[1][1] =	1;\
	c[1][2] =	0;\
	c[1][3] =	0;\
	c[2][0] =	-sq;\
	c[2][1] =	0;\
	c[2][2] =	cq;\
	c[2][3] =	0;\
	c[3][0] =	0;\
	c[3][1] =	0;\
	c[3][2] =	0;\
	c[3][3] =	1;\

#define MTX_ROZ \
	c[0][0] =	cq;\
	c[0][1] =	sq;\
	c[0][2] =	0;\
	c[0][3] =	0;\
	c[1][0] =	-sq;\
	c[1][1] =	cq;\
	c[1][2] =	0;\
	c[1][3] =	0;\
	c[2][0] =	0;\
	c[2][1] =	0;\
	c[2][2] =	1;\
	c[2][3] =	0;\
	c[3][0] =	0;\
	c[3][1] =	0;\
	c[3][2] =	0;\
	c[3][3] =	1;\

/* C(4x4)=I will be changed to cos(q),... in respective values */
#define PI (3.1415926535897932384L)
#define r2d(q) ((q) *(180.0L/PI)) 
#define d2r(q) ((q) *(PI/180.0L)) 
void  mtxf_rox(const float q, float *C)
{
	float (*c)[4] = (float (*)[4])C;
	float cq = cos(q);
	float sq = sin(q);
	MTX_ROX;
}
void  mtxf_roy(const float q, float *C)
{
	float (*c)[4] = (float (*)[4])C;
	float cq = cos(q);
	float sq = sin(q);
	MTX_ROY;
}
void  mtxf_roz(const float q, float *C)
{
	float (*c)[4] = (float (*)[4])C;
	float cq = cos(q);
	float sq = sin(q);
	MTX_ROZ;
}
void mtxlf_rox(const double q, double *C)
{
	double (*c)[4] = (double (*)[4])C;
	double cq = cosl(q);
	double sq = sinl(q);
	MTX_ROX;
}
void mtxlf_roy(const double q, double *C)
{
	double (*c)[4] = (double (*)[4])C;
	double cq = cosl(q);
	double sq = sinl(q);
	MTX_ROY;
}
void mtxlf_roz(const double q, double *C)
{
	double (*c)[4] = (double (*)[4])C;
	double cq = cos(q);
	double sq = sin(q);
	MTX_ROZ;
}
void mtxLf_rox(const long double q, long double *C)
{
	long double (*c)[4] = (long double (*)[4])C;
	long double cq = cosl(q);
	long double sq = sinl(q);
	MTX_ROX;
}
void mtxLf_roy(const long double q, long double *C)
{
	long double (*c)[4] = (long double (*)[4])C;
	long double cq = cosl(q);
	long double sq = sinl(q);
	MTX_ROY;
}
void mtxLf_roz(const long double q, long double *C)
{
	long double (*c)[4] = (long double (*)[4])C;
	long double cq = cosl(q);
	long double sq = sinl(q);
	MTX_ROZ;
}
#undef PI
#undef r2d
#undef MTX_ROX
#undef MTX_ROY
#undef MTX_ROZ
#endif /* MPU_MTX_ROT_H */
