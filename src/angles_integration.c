// SPDX-License-Identifier: MIT
/* Copyright (C) 2021 Thales Antunes de Oliveira Barretto */
#include "angles.h"
#include <libmtx/mtx.h>
#include <tgmath.h>

int angles_integrate_trapezoidal(struct mpu_ang *base, struct mpu_ang *ang)
{
	/* sampling period */
	long double T = ang->dev->st;

	/* rate Gyro data - from degs/s to radians/s */
	long double P =	 d2r(*(ang->dev->Gx));
	long double Q =	-d2r(*(ang->dev->Gy));
	long double R =	-d2r(*(ang->dev->Gz));

	/* Phi[k-1] previous sample attitude  - from degs to radians */
	long double phi	  = d2r(base->ean[0]);
	long double theta = d2r(base->ean[1]);
	long double psi	  = d2r(base->ean[2]);

	/* uses Table 2.5-1 Formulas */
	long double phi_dot	= P + tan(Q * sin(phi) + R * cos(phi));
	long double theta_dot	= Q * cos(phi) - R * sin(phi);
	long double psi_dot	= ((Q * sin(phi)) + (R * cos(phi))) / cos(theta);

	/* first order integration */
	long double phi_euler	= phi	+ (T * phi_dot);
	long double theta_euler	= theta	+ (T * theta_dot);
	long double psi_euler	= psi	+ (T * psi_dot);

	/* re-evaluate rate in radians/s */
	long double phi_euler_dot	= phi_euler	- phi;
	long double theta_euler_dot	= theta_euler	- theta;
	long double psi_euler_dot	= psi_euler	- psi;

	/* find Phi[k] by second order rate estimation, send back to degs */
	ang->ean[0] = r2d(phi	+ ((T*(phi_euler_dot	+ phi_dot	))/2.L));
	ang->ean[1] = r2d(theta	+ ((T*(theta_euler_dot	+ theta_dot	))/2.L));
	ang->ean[2] = r2d(psi	+ ((T*(psi_euler_dot	+ psi_dot	))/2.L));

	return 0;
}

int angles_integrate_euler(struct mpu_ang *base, struct mpu_ang *ang)
{
	/* sampling period */
	long double T = ang->dev->st;

	/* rate Gyro data - from degs/s to radians/s */
	long double P =	 d2r(*(ang->dev->Gx));
	long double Q =	-d2r(*(ang->dev->Gy));
	long double R =	-d2r(*(ang->dev->Gz));

	/* Phi[k-1] previous sample attitude  - from degs to radians */
	long double phi	  = d2r(base->ean[0]);
	long double theta = d2r(base->ean[1]);
	long double psi	  = d2r(base->ean[2]);

	/* uses Table 2.5-1 Formulas */
	long double phi_dot	= P + tan(Q * sin(phi) + R * cos(phi));
	long double theta_dot	= Q * cos(phi) - R * sin(phi);
	long double psi_dot	= ((Q * sin(phi)) + (R * cos(phi))) / cos(theta);

	/* first order integration */
	ang->ean[0] = r2d(phi	+ (T * phi_dot));
	ang->ean[1] = r2d(theta + (T * theta_dot));
	ang->ean[2] = r2d( psi	+ (T * psi_dot));

	return 0;
}

/* first order integration using matrix multiplication */
int angles_integrate_matrix(struct mpu_ang *base, struct mpu_ang *ang)
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
