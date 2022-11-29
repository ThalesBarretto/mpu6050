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
	 * UPDATE: this may be dua to aliasing behaviour.
	 * TODO: try -fno-strict-aliasing
	 * TODO: try __attribute__((noinline))
	 * TODO: discussion <https://stackoverflow.com/questions/2958633/gcc-strict-aliasing-and-horror-stories>
	 */
	long double omega[3];	// PQR gyro rates
	long double phi[3];	// Euler last angles
	long double phi_dot[3]; // Euler angle rates
	long double phi_inc[3]; // Euler angle increment over period T
	long double r_old[9];	// Rotation matrix for old attitude
	long double r_inc[9];	// Rotation matrix for increment
	long double r_new[9];	// Rotation matrix for new attitude
	long double phi_new[3];	// Euler new angles

	/* 1st, find omega from gyro */
	omega[0] =  d2r(*(ang->dev->Gx));
	omega[1] = -d2r(*(ang->dev->Gy));
	omega[2] = -d2r(*(ang->dev->Gz));
	/* 2d, take last ypr angles */
	phi[0] = d2r(base->ean[0]);
	phi[1] = d2r(base->ean[1]);
	phi[2] = d2r(base->ean[2]);

	mtx_rot_ypr_wtp(phi, omega, phi_dot); 			/*  find phi_dot */
	mtx_mul_scl(phi_dot, 3, 1, ang->dev->st, phi_inc); 	/*  find increment, assume constant rate */
	mtx_rot_ypr_etr(phi, r_old); 				/*  find rotation matrix for old angles */
	mtx_rot_ypr_etr(phi_inc, r_inc); 			/*  find rotation matrix for increment */
	mtx_mul(r_inc, r_old, 3,3,3, r_new); 			/*  find resulting rotation matrix */
	mtx_rot_ypr_rte(r_new, phi_new); 			/*  find euler angles from new matrix */

	/* back to radians, just cause... */
	ang->ean[0] = r2d(phi_new[0]);
	ang->ean[1] = r2d(phi_new[1]);
	ang->ean[2] = r2d(phi_new[2]);
	/*  __HIC SUNT DRACONES__ -- END */

	return 0;
}

/*  Estimate angles from accel */
#ifndef square
#define square(x) ((x) * (x))
#endif
int angles_accel(struct mpu_ang *ang)
{
	if (NULL == ang) {
		fprintf(stderr, "%s - Error: paramter base == NULL\n", __func__);
		return -1;
	}

	/* ASSUME ang is provided in DEGREES  */
	ang->rho =  r2d((atan2 (sqrt(*(ang->dev->Ax2) + *(ang->dev->Ay2)), -*(ang->dev->Az))));
	ang->ean[0] = r2d((atan2(-1.L* *(ang->dev->Ay), -1.L * *(ang->dev->Az)))); /* invert phi to express ypr(frd frame) */
	ang->ean[1] = r2d((atan2(-1.L* *(ang->dev->Ax), sqrt(square(-1.L * *(ang->dev->Ay)) + square(-1.L * *(ang->dev->Az))) ))); /* invert theta to express ypr(frd frame) */
	ang->ean[2] = 0;	/* accel can't provide psi */
	return 0;
}

