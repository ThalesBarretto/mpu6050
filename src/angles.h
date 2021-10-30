#ifdef __cplusplus
	extern "C" {
#endif
#ifndef _ANGLES_H_
#define _ANGLES_H_

#include <libmpu6050/mpu6050_core.h>

struct mpu_ang {
	struct mpu_dev *dev;
	int isd;		/* 0 for radians, 1 for degrees 		*/
	long double  rho;    	/* tilt angle					*/
	long double  ean[3];	/* euler angles (phi, theta, psi) 		*/
	long double  axa[4];	/* axis(x,y,z,theta) for (phi, theta, psi)	*/
	long double  qua[4];	/* quaternion for (phi, theta, psi		*/
	long double  rtm[9];	/* rotation matrix for (phi, theta, psi)	*/
};

struct mpu_anr {
	struct mpu_dev *dev;
	int isd;		/* 0 for radians, 1 for degrees 		*/
	long double  rhr;    	/* tilt angle 					*/
	long double  ear[3];	/* euler angles rates  (phi, theta, psi) 	*/
	long double  qur[9];	/* quaternion rate 				*/
	long double  cpm[9];	/* cross product matrix 			*/
	long double  cp2[9];	/* cross product matrix square 			*/
};

int angles_init(struct mpu_dev *, struct mpu_ang **ang);
int angles_destroy(struct mpu_ang * ang);
int angles_accel(struct mpu_ang * ang);
int angles_gyro(struct mpu_ang * base, struct mpu_ang *ang);
#endif /* _ANGLES_H_ */
#ifdef __cplusplus
	}
#endif
