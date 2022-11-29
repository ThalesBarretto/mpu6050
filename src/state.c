// SPDX-License-Identifier: MIT
/* Copyright (C) 2021 Thales Antunes de Oliveira Barretto */
#include "state.h"
#include "state_math.h"
#include <tgmath.h>
#include <string.h>
#include <stdio.h>

void series_integrate_last(series_t *S)
{
	state_integrate_trapezoidal(&(S->last->X), S->imu->cfg.Fs, S->integ.steps, S->imu->gforce);
}

/* goes from S[k-1] to S[k] */
void state_integrate_trapezoidal(state_t *S, mpu_data_t Fs, size_t steps, long double gforce)
{
	long double T = 1/(steps *Fs); /* step period */
	for(size_t i = 0; i < steps ; i++) {
		S->dot_phi	= S->P + tan(S->Q * sin(S->phi) + S->R * cos(S->phi));
		S->dot_theta	= S->Q * cos(S->phi) - S->R * sin(S->phi);
		S->dot_psi	= ((S->Q * sin(S->phi)) + (S->R * cos(S->phi))) / cos(S->theta);
		long double phi_euler	= S->phi	+ (T * S->dot_phi	);
		long double theta_euler	= S->theta	+ (T * S->dot_theta	);
		long double dot_phi_euler	= S->P + tan(S->Q * sin(phi_euler) + S->R * cos(phi_euler));
		long double dot_theta_euler	= S->Q * cos(phi_euler) - S->R * sin(phi_euler);
		long double dot_psi_euler	= ((S->Q * sin(phi_euler)) + (S->R * cos(phi_euler))) / cos(theta_euler);
		S->phi	 += ((T/2.L)*(dot_phi_euler	+ S->dot_phi	));
		S->theta += ((T/2.L)*(dot_theta_euler	+ S->dot_theta	));
		S->psi	 += ((T/2.L)*(dot_psi_euler	+ S->dot_psi	));
		S->psi	 = fmodl(S->phi, M_PI);
		S->psi	 = fmodl(S->theta, (M_PI/2.L));
		S->psi	 = fmodl(S->psi, M_PI);
		S->an =   S->fx * cos(S->psi) * cos(S->theta) \
			+ S->fy * (-sin(S->phi) * sin(S->theta) * cos(S->psi) + sin(S->psi) * cos(S->phi)) \
			+ S->fz * (-sin(S->phi) * sin(S->psi) - sin(S->theta) * cos(S->phi) * cos(S->psi)) ;
		S->ae =   S->fx * sin(S->psi) * cos(S->theta) \
			+ S->fy * (-sin(S->phi) * sin(S->psi) * sin(S->theta) - cos(S->phi) * cos(S->psi)) \
			+ S->fz * ( sin(S->phi) * cos(S->psi) - sin(S->psi) * sin(S->theta) * cos(S->phi)) ;
		S->au = - S->fx * sin(S->theta) \
				 - S->fy * sin(S->phi) * cos(S->theta) \
				 - S->fz * cos(S->phi) * cos(S->theta) - gforce;
		S->vn += ( S->an * T);
		S->ve += ( S->ae * T);
		S->vu += ( S->au * T);
		S->pn += S->vn * T;
		S->pe += S->ve * T;
		S->pu += S->vu * T;
	}
}

void set_Cfrdned(data_t Cnedfrd[3][3], state_t *S)
{
/* ETR - Euler angles to rotation matrix
 *  JOHNSON:	Cfrd/ned		(eq. 1.3-10) EULER  to MATRIX
 *  DIEBEL:	R_123(phi,theta,psi)	(eq.287) EULER to MATRIX 
 */
	data_t c_phi = cos(S->phi); 
	data_t c_the = cos(S->theta); 
	data_t c_psi = cos(S->psi); 
	data_t s_phi = sin(S->phi); 
	data_t s_the = sin(S->theta); 
	data_t s_psi = sin(S->psi); 
	Cnedfrd[0][0] = ( c_the * c_psi );
	Cnedfrd[0][1] = ( c_the * s_psi );
	Cnedfrd[0][2] = (-s_the );
	Cnedfrd[1][0] = ( s_phi * s_the * c_psi) - ( c_phi * s_psi );
	Cnedfrd[1][1] = ( s_phi * s_the * s_psi) + ( c_phi * c_psi );
	Cnedfrd[1][2] = ( s_phi * c_the );
	Cnedfrd[2][0] = ( c_phi * s_the * c_psi) + ( s_phi * s_psi );
	Cnedfrd[2][1] = ( c_phi * s_the * s_psi) - ( s_phi * c_psi );
	Cnedfrd[2][2] = ( c_phi * c_the );
}

void set_Cnedfrd(data_t Cnedfrd[3][3], state_t *S)
{
/* BTR - Euler angles to reverse rotation matrix
 *  HENDERSON: MZYX(3,2,1) (seq. 10)  (1=phi,2=theta,3=psi)
 */
	data_t c_phi = cos(S->phi);
	data_t c_the = cos(S->theta);
	data_t c_psi = cos(S->psi);
	data_t s_phi = sin(S->phi);
	data_t s_the = sin(S->theta);
	data_t s_psi = sin(S->psi);
	Cnedfrd[0][0] = ( c_psi * c_the );
	Cnedfrd[0][1] = ( s_phi * s_the * c_psi) - ( s_psi * c_phi);
	Cnedfrd[0][2] = ( s_the * c_phi * c_psi) + ( s_phi * s_psi);
	Cnedfrd[1][0] = ( s_psi * c_the );
	Cnedfrd[1][1] = ( s_phi * s_psi * s_the) + ( c_phi * c_psi);
	Cnedfrd[1][2] = (-s_phi * c_psi) + ( s_psi * s_the * c_phi);
	Cnedfrd[2][0] = (-s_the);
	Cnedfrd[2][1] = ( s_phi * c_the);
	Cnedfrd[2][2] = ( c_phi * c_the);
}

void set_PHI_dot(state_t *S)
{
	S->dot_phi	= S->P + tan(S->Q * sin(S->phi) + S->R * cos(S->phi));
	S->dot_theta	= S->Q * cos(S->phi) - S->R * sin(S->phi);
	S->dot_psi	= ((S->Q * sin(S->phi)) + (S->R * cos(S->phi))) / cos(S->theta);
}

void snprint_state(state_t *S, char *msg, char *buf)
{
	/* derivatives */
	//sprintf(buf, "dph:%+.2Lf ",	r2d(S->dot_phi));	strcat(msg, buf);
	//sprintf(buf, "dth:%+.2Lf ",	r2d(S->dot_theta));	strcat(msg, buf);
	//sprintf(buf, "dps:%+.2Lf ",	r2d(S->dot_psi));	strcat(msg, buf);
	//sprintf(buf, "P:%+.2Lf ",	S->P);	strcat(msg, buf);
	//sprintf(buf, "Q:%+.2Lf ",	S->Q);	strcat(msg, buf);
	//sprintf(buf, "R:%+.2Lf ",	S->R);	strcat(msg, buf);
	sprintf(buf, "phi:%+.2Lf ",	r2d(S->phi));	strcat(msg, buf);
	sprintf(buf, "the:%+.2Lf ",	r2d(S->theta));	strcat(msg, buf);
	sprintf(buf, "psi:%+.2Lf ",	r2d(S->psi));	strcat(msg, buf);
	//sprintf(buf, "vn:%+.3Lf ",	S->vn);	strcat(msg, buf);
	//sprintf(buf, "ve:%+.3Lf ", 	S->ve);	strcat(msg, buf);
	//sprintf(buf, "vu:%+.3Lf ", 	S->vu);	strcat(msg, buf);
	sprintf(buf, "pn:%+.3Lf ",	S->pn);	strcat(msg, buf);
	sprintf(buf, "pe:%+.3Lf ",	S->pe);	strcat(msg, buf);
	sprintf(buf, "pu:%+.3Lf ",	S->pu);	strcat(msg, buf);
	//sprintf(buf, "fx:%+.5Lf ",	S->fx);	strcat(msg, buf);
	//sprintf(buf, "fy:%+.5Lf ",	S->fy); strcat(msg, buf);
	//sprintf(buf, "fz:%+.5Lf ",	S->fz); strcat(msg, buf);
	//sprintf(buf, "an:%+.5Lf ",	S->an);	strcat(msg, buf);
	//sprintf(buf, "ae:%+.5Lf ",	S->ae);	strcat(msg, buf);
	//sprintf(buf, "au:%+.5Lf ",	S->au);	strcat(msg, buf);

}

void imu_get_data(imu_t *imu,struct mpu_dev *dev)
{
		imu->dat.Gx= d2r(*(dev->Gx));
		imu->dat.Gy= d2r(*(dev->Gy));
		imu->dat.Gz= d2r(*(dev->Gz));
		imu->dat.Ax= imu->gforce * *(dev->Ax);
		imu->dat.Ay= imu->gforce * *(dev->Ay);
		imu->dat.Az= imu->gforce * *(dev->Az);
}
