#ifndef PI
#ifdef M_PI
#define PI M_PI
#else
#define PI (3.1415926535897932384)
#define M_PI PI
#endif
#endif

#ifndef square
#define square(x) ((x) * (x))
#endif

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

static inline float __attribute__((always_inline)) d2rf(const float q)
{
	return q * mtx_rot_d2rf;
}
static inline double __attribute__((always_inline)) d2rlf(const double q)
{
	return q * mtx_rot_d2rl;
}
static inline long double __attribute__((always_inline)) d2rLf(const long double q)
{
	return q * mtx_rot_d2rLf;
}
static inline float __attribute__((always_inline)) r2df(const float q)
{
	return q * mtx_rot_r2df;
}
static inline double __attribute__((always_inline)) r2dlf(const double q)
{
	return q * mtx_rot_r2dl;
}
static inline long double __attribute__((always_inline)) r2dLf(const long double q)
{
	return q * mtx_rot_r2dLf;
}
