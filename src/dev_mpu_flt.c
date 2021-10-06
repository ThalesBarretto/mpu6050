#include "dev_mpu_flt.h"

/* get angle data from accelerometer*/
static inline void mpu_ang_acc(struct mpu_ang * ang);
/* get angle data from gyro */
static inline void mpu_ang_gyr(struct mpu_ang * base, struct mpu_ang *ang);

void mpu_ang_init(struct mpu_dev * dev, struct mpu_ang **ang)
{
	*ang = (struct mpu_ang *)calloc(1, sizeof(struct mpu_ang));
	assert(NULL != *ang);

	(*ang)->dev = dev;
	assert(NULL != (*ang)->dev);

	/* assume degrees */
	(*ang)->isd = 1;
}

/*  integrate angles from gyro rates */
static inline void mpu_ang_gyr(struct mpu_ang *base, struct mpu_ang *ang) 
{
	/* TODO 
	 * use the function 
	 * 	mpu_mtx_rot_ypr_wtp(Phi, Omega, Phi_dot);
	 */
	long double Omega[3];
	long double Phi[3];
	long double Phi_dot[3] = {0,0,0};

	/* first, take gyro in body_xyz frame */
	Omega[0] = d2r(*(ang->dev->Gx));
	Omega[1] = d2r(*(ang->dev->Gy));
	Omega[2] = d2r(*(ang->dev->Gz));

	/* second, bring to body_frd frame */
	Omega[1] *= -1;
	Omega[2] *= -1; 

	/* third, take last ypr angles */
	Phi[0] = d2r(base->ean[0]);
	Phi[1] = d2r(base->ean[1]);
	Phi[2] = d2r(base->ean[2]);

	mtx_rot_ypr_wtp(Phi, Omega, Phi_dot);

	/* assume constant rate */
	ang->ean[0] = (r2d(Phi_dot[0]) * ang->dev->st);
	ang->ean[1] = (r2d(Phi_dot[1]) * ang->dev->st);
	ang->ean[2] = (r2d(Phi_dot[2]) * ang->dev->st);
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
	ang->rho =  r2d((atan2l (sqrtl(*(ang->dev->Ax2) + *(ang->dev->Ay2)), -*(ang->dev->Az))));
	ang->ean[0] = -r2d((atan2l(1.L* *(ang->dev->Ay), -1.L * *(ang->dev->Az))));
	ang->ean[1] = -r2d((atan2l(1.L* *(ang->dev->Ax), -1.L * *(ang->dev->Az))));
	ang->ean[2] = 0;
}

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
	(*flt)->mtr = 0.5L;		/* accel motion treshold in g's	*/
	(*flt)->ant = 0.08L;		/* accel noise treshold in g's	*/

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
	if (fabs((*(flt->dev->AM) - flt->dev->cal->gra) > flt->mtr)) {
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

void mpu_ang_pri(struct mpu_ang *ang, char *msg, char *buf)
{
	sprintf(buf, "phi:%+.2Lf ", ang->ean[0]);
	strcat(msg, buf);

	sprintf(buf, "the:%+.2Lf ", ang->ean[1]);
	strcat(msg, buf);

	sprintf(buf, "psi:%+.2Lf ", ang->ean[2]);
	strcat(msg, buf);
}


