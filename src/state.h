// SPDX-License-Identifier: MIT
/* Copyright (C) 2021 Thales Antunes de Oliveira Barretto */
#ifdef __cplusplus
	extern "C" {
#endif
#ifndef _STATE_H_
#define _STATE_H_

#include <libmpu6050/mpu6050_core.h>
#include <tgmath.h>

#define MAXLINE 1024ul
#define MAXNAME 32ul
#define gD 9.80665L

/* public */
typedef long double data_t;
typedef struct vec3_t vec3_t;
typedef struct mtx3x3_t mtx3x3_t;
typedef struct attitude_t attitude_t;
typedef struct position_t position_t;
typedef struct relpos_t relpos_t;
typedef struct imu_t imu_t;
typedef struct imu_cfg_t imu_cfg_t;
typedef struct imu_data_t imu_data_t;
typedef struct series_t series_t;
typedef struct state_t state_t;
typedef struct record_t record_t;

typedef struct integration_t integration_t;
typedef enum method_t {
	RK_EULER  = 1,
	RK_TRAPEZ = 2
} method_t ;

typedef struct frame_t {
	char name[MAXNAME];
	char code[3];
	/* angular velocity w/r */
	data_t P;
	data_t Q;
	data_t R;
}frame_t;

struct integration_t {
	size_t steps;
	method_t method;
};

struct vec3_t {
	data_t v1;
	data_t v2;
	data_t v3;
};

struct attitude_t {
	union {
		vec3_t Phi;
		struct {
			data_t phi;
			data_t theta;
			data_t psi;
		};
	};
};

typedef struct  axis_t {
	char name[MAXNAME];
	char code[3];
	frame_t frame;
	attitude_t att;
	enum {
	AXIS_XYZ = 1, /* standard right-handed coordinate system */
	AXIS_FRD = 2, /* body-fixed CCW front-right-down */
	AXIS_NED = 3, /* earth-fixed CCW North-East-down */
	AXIS_NEH = 4  /* tangent-plane CCW North-East-Up */
	} type ;
} axis_t ;

struct mtx3x3_t {
	union {
		vec3_t col1 ;
	       struct {
		data_t c11;
		data_t c21;
		data_t c31;
		};
	};
	union {
		vec3_t col2 ;
	       struct {
		data_t c12;
		data_t c22;
		data_t c32;
		};
	};
	union {
		vec3_t col3 ;
	       struct {
		data_t c23;
		data_t c13;
		data_t c33;
	       };
	};
};

struct position_t {
	union {
		vec3_t p ;
	       struct {
		data_t p1;
		data_t p2;
		data_t p3;
	       };
	};
};

struct relpos_t {
	position_t *reference;
	position_t pos;
	attitude_t att;
};

struct imu_data_t {
	data_t Ax;
	data_t Ay;
	data_t Az;
	data_t Gx;
	data_t Gy;
	data_t Gz;
};

struct imu_cfg_t {
	data_t Accel_range;
	data_t Gyro_range;
	data_t Fs;
};

struct imu_t {
	imu_cfg_t cfg;
	imu_data_t dat;
	relpos_t relpos; /* pose relative to center of mass */
	long double gforce;
	char description[MAXLINE];
};

typedef struct state_t {
	/* vector X = { PHI[3] p[3] }---------------------------*/
	data_t  phi;		/* PHI[0] roll angle		*/
	data_t  theta;		/* PHI[1] pitch angle		*/
	data_t  psi;		/* PHI[3] yaw angle		*/
	data_t  pn;		/* p[0] positive north		*/
	data_t  pe;		/* p[1] positive east		*/
	data_t  pu;		/* p[2] positive down		*/
	/* vector Xdot = { PHI_dot[3] p_dot[3] }----------------*/
	data_t  dot_phi;	/* PHI_dot[0]			*/
	data_t  dot_theta;	/* PHI_dot[1]			*/
	data_t  dot_psi;	/* PHI_dot[2]			*/
	data_t  vn;		/* p_dot[0]			*/
	data_t  ve;		/* p_dot[1]			*/
	data_t  vu;		/* p_dot[2]			*/
	/* vector u = { fxyz omega } -------------------------- */
	data_t  fx;	/* f[0]					*/
	data_t  fy;	/* f[1]					*/
	data_t  fz;	/* f[2]					*/
	data_t  P;	/* omega[0]				*/
	data_t  Q;	/* omega[1]				*/
	data_t  R;	/* omega[2]				*/
} state_t;

struct record_t {
	record_t *prev;
	record_t *next;
	size_t	count;
	state_t X;
};


struct series_t {
	record_t *first;
	record_t *last;
	record_t *pos;
	char description[MAXLINE];
	size_t size;
	integration_t integ;
	imu_t *imu;
} ;

void set_f(state_t *S, struct mpu_dev *dev);
void set_omega(state_t *S, struct mpu_dev *dev);
void set_Cfrdned(data_t Cfrdned[3][3], state_t *S);
void set_Cnedfrd(data_t Cnedfrd[3][3], state_t *S);
void set_PHI_dot(state_t *S);

state_t state_zero(void);

void series_init(series_t *Series, char * Description, imu_t * Imu, const integration_t * Integration, size_t Size);

void series_test(void);
void snprint_state(state_t *S, char *msg, char *buf);

state_t* state_new(size_t many);
record_t* record_new(size_t many);
series_t* series_new(size_t many);
void record_push(series_t *Series, record_t *Record);
void state_integrate_trapezoidal(state_t *S, mpu_data_t Fs, size_t steps, long double gforce);
void series_integrate_last(series_t *S);
#endif /* _STATE_H_ */
#ifdef __cplusplus
	}
#endif
