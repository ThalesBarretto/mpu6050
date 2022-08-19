// SPDX-License-Identifier: MIT
/* Copyright (C) 2021 Thales Antunes de Oliveira Barretto */
#include <stdlib.h>
#include <stdio.h>
#include <tgmath.h>
#include <libmpu6050/mpu6050_core.h>
#include <libmtx/mtx.h>
#include "angles.h"

int angles_init(struct mpu_dev * dev, struct mpu_ang **ang)
{
	if(NULL == dev)
		fprintf(stderr, "%s - dev is NULL\n", __func__);

	*ang = (struct mpu_ang *)calloc(1, sizeof(struct mpu_ang));
	(*ang)->dev = dev;
	(*ang)->isd = 1; /* assume degrees */

	return 0;
}

int angles_destroy(struct mpu_ang *ang)
{
	if(NULL != ang)
		free(ang);
	return 0;
}

/*  integrate angles from gyro rates */
int angles_gyro(struct mpu_ang *base, struct mpu_ang *ang)
{
	if (NULL == base) {
		fprintf(stderr, "%s - Error: paramter base == NULL\n", __func__);
		return -1;
	}
	if (NULL == ang){
		fprintf(stderr, "%s - Error: paramter base == NULL\n", __func__);
		return -1;
	}

	/*  __HIC SUNT DRACONES__ -- BEGIN
	 *
	 *  PLEASE DON'T CHANGE IT
	 *  For some reason the FPU will misbehave
	 *
	 */
	long double Omega[3];	// PQR gyro rates
	long double Phi[3];	// Euler last angles
	long double Phi_dot[3]; // Euler angle rates
	long double Phi_inc[3]; // Euler angle increment over period T
	long double R_old[9];	// Rotation matrix for old attitude
	long double R_inc[9];	// Rotation matrix for increment
	long double R_new[9];	// Rotation matrix for new attitude
	long double Phi_new[3];	// Euler new angles

	/* 1st, find Omega from gyro */
	Omega[0] =  d2r(*(ang->dev->Gx));
	Omega[1] = -d2r(*(ang->dev->Gy));
	Omega[2] = -d2r(*(ang->dev->Gz));
	/* 2d, take last ypr angles */
	Phi[0] = d2r(base->ean[0]);
	Phi[1] = d2r(base->ean[1]);
	Phi[2] = d2r(base->ean[2]);

	mtx_rot_ypr_wtp(Phi, Omega, Phi_dot); 			/*  find Phi_dot */
	mtx_mul_scl(Phi_dot, 3, 1, ang->dev->st, Phi_inc); 	/*  find increment, assume constant rate */
	mtx_rot_ypr_etr(Phi, R_old); 				/*  find rotation matrix for old angles */
	mtx_rot_ypr_etr(Phi_inc, R_inc); 			/*  find rotation matrix for increment */
	mtx_mul(R_inc, R_old, 3,3,3, R_new); 			/*  find resulting rotation matrix */
	mtx_rot_ypr_rte(R_new, Phi_new); 			/*  find euler angles from new matrix */

	/* back to radians, just cause... */
	ang->ean[0] = r2d(Phi_new[0]);
	ang->ean[1] = r2d(Phi_new[1]);
	ang->ean[2] = r2d(Phi_new[2]);
	/*  __HIC SUNT DRACONES__ -- END */

	return 0;
}

/*  Estimate angles from accel */
int angles_accel(struct mpu_ang *ang)
{
	if (NULL == ang) {
		fprintf(stderr, "%s - Error: paramter base == NULL\n", __func__);
		return -1;
	}

	/* ASSUME ang is provided in DEGREES  */
	ang->rho =  r2d((atan2 (sqrt(*(ang->dev->Ax2) + *(ang->dev->Ay2)), -*(ang->dev->Az))));
	ang->ean[0] = -r2d((atan2(1.L* *(ang->dev->Ay), -1.L * *(ang->dev->Az)))); /* invert phi to express ypr(frd frame) */
	ang->ean[1] = -r2d((atan2(1.L* *(ang->dev->Ax), -1.L * *(ang->dev->Az)))); /* invert theta to express ypr(frd frame) */
	ang->ean[2] = 0;	/* accel can't provide psi */
	return 0;
}

