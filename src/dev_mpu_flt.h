#ifdef __cplusplus
	extern "C" {
#endif

#ifndef MPU_FLT_H_
#define MPU_FLT_H_

#include "include/mpu6050_driver.h"
#include "mtx.h"
#include "mtx_mul.h"
#include "mtx_rot.h"
#include "mtx_rot_ypr.h"
#include <assert.h>
#include <inttypes.h>
#define MDEBUG printf("%3d %-12s\n", __LINE__, __func__);

/* angles set */
struct mpu_ang {
	struct mpu_dev *dev;
	int isd;		/* 0 for radians, 1 for degrees 		*/
	long double  rho;    	/* tilt angle					*/
	long double  ean[3];	/* euler angles (phi, theta, psi) 		*/
	long double  axa[4];	/* axis(x,y,z,theta) for (phi, theta, psi)	*/
	long double  qua[4];	/* quaternion for (phi, theta, psi		*/
	long double  rtm[9];	/* rotation matrix for (phi, theta, psi)	*/
};

/* angle rates set */
struct mpu_anr {
	struct mpu_dev *dev;
	int isd;		/* 0 for radians, 1 for degrees 		*/
	long double  rhr;    	/* tilt angle 					*/
	long double  ear[3];	/* euler angles rates  (phi, theta, psi) 	*/
	long double  qur[9];	/* quaternion rate 				*/
	long double  cpm[9];	/* cross product matrix 			*/
	long double  cp2[9];	/* cross product matrix square 			*/
};

/* filter data set */
struct mpu_flt_dat {
	struct mpu_dev *dev;	/* device from which data is taken	*/
	struct mpu_ang *anf;	/* filtered angles			*/
	struct mpu_ang *anr;	/* rates from filtered angles		*/

	struct mpu_ang *ana;	/* accel angles				*/
	struct mpu_ang *ang;	/* gyro integration angles		*/
	struct mpu_ang *anm;	/* mag angles				*/

	/* optional parameters */
	int isd;		/* 0 for radians, 1 for degrees		*/
	long double mtr;	/* motion treshold acceleration in g's	*/
	long double ant;	/* axis acceleration noise treshold	*/ 
	long double gain;	/* general parameter "gain		*/
	long double gaa;	/* computed gain for accelerometer	*/
	long double gag;	/* computed gain for gyroscope		*/
	long double gam;	/* computed gain for magnetometer	*/
};

/* initializes a struct mpu_ang */
void mpu_ang_init(struct mpu_dev *, struct mpu_ang **ang);

/* initializes a struct mpu_flt_dat */
void mpu_flt_com_init(struct mpu_dev *dev, struct mpu_flt_dat **flt);

/* prints a struct mpu_ang */
void mpu_ang_pri(struct mpu_ang *ang, char *msg, char *buf);

/* updates the filter */
void mpu_flt_com_update(struct mpu_flt_dat *flt);
/* prints the angles from struct mpu_flt_dat */
void mpu_flt_ang_pri(struct mpu_flt_dat *flt);

#endif /* MPU_FLT_H_ */

#ifdef __cplusplus
	}
#endif /* __cplusplus */
