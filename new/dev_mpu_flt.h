#ifdef __cplusplus
	extern "C" {
#endif

#ifndef MPU_FLT_H_
#define MPU_FLT_H_

#include "dev_mpu.h"
#include "dev_mpu_mtx.h"
#include <assert.h>
#include <inttypes.h>

#define MDEBUG printf("%3d %-12s\n", __LINE__, __func__);
#define PI (3.1415926535897932384L)
#define r2d (180.0L / PI) 
#define d2r (PI / 180.0L) 

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

void mpu_ang_init(struct mpu_dev *, struct mpu_ang **ang);
static inline void mpu_ang_acc(struct mpu_ang * ang);
static inline void mpu_ang_gyr(struct mpu_ang * base, struct mpu_ang *ang);
void mpu_ang_pri(struct mpu_ang *ang);

void mpu_ang_init(struct mpu_dev * dev, struct mpu_ang **ang)
{
	*ang = (struct mpu_ang *)calloc(1, sizeof(struct mpu_ang));
	assert(NULL != *ang);

	(*ang)->dev = dev;
	assert(NULL != (*ang)->dev);

	/* assume degrees */
	(*ang)->isd = 1;
}

/*  ASSUME DEGREES integrate gyro angle change from gyro rates */
static inline void mpu_ang_gyr(struct mpu_ang *base, struct mpu_ang *ang) 
{
	/* assume constant rate */
	ang->ean[0] = (*(ang->dev->Gx) * ang->dev->st);
	ang->ean[1] = (*(ang->dev->Gy) * ang->dev->st);
	ang->ean[2] = (*(ang->dev->Gz) * ang->dev->st);
	/* add to base */
	if ((base != NULL) && (base != 0)) {
		ang->ean[0] += base->ean[0];
		ang->ean[1] += base->ean[1];
		ang->ean[2] += base->ean[2];
	}

	//TODO: find formula for rho
	//ang->rho = 0;
}

/* ASSUME DEGREES - Estimate angles from accel */
static inline void mpu_ang_acc(struct mpu_ang *ang)
{
	ang->rho =  r2d * (atan2l (sqrtl(*(ang->dev->Ax2) + *(ang->dev->Ay2)), -*(ang->dev->Az)));
	ang->ean[0] = -r2d * (atan2l(1.L* *(ang->dev->Ay), -1.L * *(ang->dev->Az)));
	ang->ean[1] = -r2d * (atan2l(1.L* *(ang->dev->Ax), -1.L * *(ang->dev->Az)));
	ang->ean[2] = 0;
}

void mpu_ang_pri(struct mpu_ang *ang)
{
	printf("phi:%+.2Lf ", ang->ean[0]);
	printf("the:%+.2Lf ", ang->ean[1]);
	printf("psi:%+.2Lf ", ang->ean[2]);
}


void mpu_flt_com_init(struct mpu_dev *dev, struct mpu_flt_dat **flt);
void mpu_flt_com_update(struct mpu_flt_dat *flt);
void mpu_flt_ang_pri(struct mpu_flt_dat *flt);

/* build a complementary filter structure */
void mpu_flt_com_init(struct mpu_dev *dev, struct mpu_flt_dat **flt)
{
	*flt = (struct mpu_flt_dat *)calloc(1, sizeof(struct mpu_flt_dat));
	assert(NULL != *flt);

	(*flt)->dev = dev;
	assert(NULL != (*flt)->dev);

	(*flt)->anf = (struct mpu_ang *)calloc(1, sizeof(struct mpu_ang));
	assert(NULL != (*flt)->anf);
	(*flt)->ana = (struct mpu_ang *)calloc(1, sizeof(struct mpu_ang));
	assert(NULL != (*flt)->ana);
	(*flt)->ang = (struct mpu_ang *)calloc(1, sizeof(struct mpu_ang));
	assert(NULL != (*flt)->ang);
	(*flt)->anm = (struct mpu_ang *)calloc(1, sizeof(struct mpu_ang));
	assert(NULL != (*flt)->anm);

	mpu_ang_init(dev, &(*flt)->anf); 
	mpu_ang_init(dev, &(*flt)->ana); 
	mpu_ang_init(dev, &(*flt)->ang); 
	mpu_ang_init(dev, &(*flt)->anm); 

	(*flt)->isd = 1;		/* output angles in degrees	*/
	(*flt)->mtr = 0.05L;		/* accel motion treshold in g's	*/
	(*flt)->ant = 0.002L;		/* accel noise treshold in g's	*/

	(*flt)->gain = 0.8L;		/* gain parameter		*/
	(*flt)->gaa = 1 - (*flt)->gain; /* computed accelerometer gain	*/
	(*flt)->gag = (*flt)->gain;	/* computed gyroscope gain	*/
}

/* update the complementary filter */
void mpu_flt_com_update(struct mpu_flt_dat *flt)
{
	/* Estimate angles from accel */
	mpu_ang_acc(flt->ana);

	/* integrate gyro rates based on last filter data */
	mpu_ang_gyr(flt->anf, flt->ang);

	/* motion detected, dont trust accelerometer */
	if (fabs((*(flt->dev->AM) -1) > flt->mtr)) {
		flt->anf->ean[0] =  flt->ang->ean[0];
		flt->anf->ean[1] =  flt->ang->ean[1];
	} else {
		/* fuse accel and gyro by complementary filter */
		flt->anf->ean[0] = (flt->gaa * flt->ana->ean[0]) + (flt->gag * flt->ang->ean[0]);
		flt->anf->ean[1] = (flt->gaa * flt->ana->ean[1]) + (flt->gag * flt->ang->ean[1]);

		//flt->anf->ean[0] = ((1 - flt->gain) * flt->ana->ean[0]) + (flt->gain * flt->ang->ean[0]);
		//flt->anf->ean[1] = ((1 - flt->gain) * flt->ana->ean[1]) + (flt->gain * flt->ang->ean[1]);
	}
	/* psi angle come only from gyro */
	flt->anf->ean[2] =  flt->ang->ean[2];

	//flt->rho =  r2d * (atan2l (sqrtl(*(flt->dev->Ax2) + *(flt->dev->Ay2)), -*(flt->dev->Az)));
	flt->anf->rho = flt->ana->rho;

}

void mpu_flt_ang_pri(struct mpu_flt_dat *flt)
{
	printf("phi:%+.2Lf ", flt->anf->ean[0]);
	printf("the:%+.2Lf ", flt->anf->ean[1]);
	printf("psi:%+.2Lf ", flt->anf->ean[2]);
	printf("rho:%+.2Lf ", flt->anf->rho);
}

#undef PI
#undef r2d

#endif /* MPU_FLT_H_ */

#ifdef __cplusplus
	}
#endif /* __cplusplus */
