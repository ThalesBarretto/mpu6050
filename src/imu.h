/* Purpose: build a device-independent frame repesentation */

/* HEADER PART */
#ifdef __cplusplus
	extern "C" {
#endif

#ifndef _IMU_H_
#define _IMU_H_

#include <stdio.h>
#include <math.h>
#include <tgmath.h>
#include "dev_mpu_mtx_rot.h"
enum rot_sequence {
	seqXYZ=1,
	seqXZY=2,
	seqXYX=3,
	seqXZX=4,
	seqYXZ=5,
	seqYZX=6,
	seqYXY=7,
	seqYZY=8,
	seqZXY=9,
	seqZYX=10,
	seqZXZ=11,
	seqZYZ=12,
	seqYPR=13
};
enum frame_type {
	frXYZ=1,
	frFRD=2,
	frNED=3,
};

//TODO: make __attribute__((packed))
struct frame {
	int type;		/* frame type	  */
	struct frame *ref;	/* reference for tracking */
	long double pos[3];		/* state - position */
	long double vel[3];		/* state - velocity */
	long double acl[3];		/* state - linear acceleration */
	long double Phi[3];		/* state - Euler angles */
	long double Omega[3];	/* state - Body-fixed angular rates */
	long double Phidot[3];	/* state - Euler angle rates */
};

struct IMU {
	void *dev;			/* abstract a device	*/
	struct frame *fr;		/* frame in which data is expressed */
	double Ts;			/* data sampling period */
	long double acc[3];			/* accel data */
	long double gyr[3];			/* gyro data */
	long double mag[3];			/* mag data */
};

void imu_estimate(struct IMU *imu);
void imu_print_Phi(struct IMU *imu);

#endif /* _IMU_H_ */

#ifdef __cplusplus
	}
#endif
