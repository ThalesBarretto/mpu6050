#ifdef __cplusplus
	extern "C" {
#endif

#ifndef MPU_FLT_H_
#define MPU_FLT_H_

#include <libmpu6050/mpu6050_core.h>
#include "angles.h"


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

int filter_init(	struct mpu_dev *dev, struct mpu_flt_dat **flt);
int filter_destroy(	struct mpu_flt_dat *flt);
int filter_update(	struct mpu_flt_dat *flt);

#endif /* MPU_FLT_H_ */

#ifdef __cplusplus
	}
#endif /* __cplusplus */
