#include <stdio.h>
/* unit converion in extra precision */
#define PI_f (3.1415926535897932384f)
#define PIlf (3.1415926535897932384 )
#define PILf (3.1415926535897932384L)
//#define  r2df(q) ((q)*(180.0f/pi_f)) 
//#define  r2dl(q) ((q)*(180.0 /pilf)) 
//#define r2dlf(q) ((q)*(180.0l/pilf)) 

static const float  		mtx_rot_r2df	= (180.0f/PI_f);
static const double  		mtx_rot_r2dl 	= (180.0 /PILf);
static const long double	mtx_rot_r2dLf	= (180.0l/PILf);
static const float  		mtx_rot_d2rf	= (PI_f/180.0f);
static const double  		mtx_rot_d2rl 	= (PILf/180.0 );
static const long double	mtx_rot_d2rLf	= (PILf/180.0l);


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

int main(void)
{
	float f = 90;
	double d = 90;
	long double l =90;
	printf("f:%6f d:%+6lf l:%+6Lf\n", f, d, l);
	f = d2r(f);
	d = d2r(d);
	l = d2r(l);
	printf("f:%6f d:%+6lf l:%+6Lf\n", f, d, l);
	f = r2d(f);
	d = r2d(d);
	l = r2d(l);
	printf("f:%6f d:%+6lf l:%+6Lf\n", f, d, l);
	//f = d2rf(f);
	//d = d2rlf(d);
	//l = d2rLf(l);

}
