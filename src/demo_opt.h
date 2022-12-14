// SPDX-License-Identifier: MIT
/* Copyright (C) 2021 Thales Antunes de Oliveira Barretto */
#ifdef __cplusplus
	extern "C" {
#endif
#ifndef _DEMO_OPT_H_
#define _DEMO_OPT_H_

#include <getopt.h>
#include <stdbool.h>
#include <libmpu6050/mpu6050_core.h>

struct  mpu_opt {
	bool quiet;	/* don't print to console	*/
	bool cs;	/* set clock source		*/
	bool dl;	/* set dlpf			*/
	bool sr;	/* set samplerate		*/
	bool ar;	/* set accel range		*/
	bool gr;	/* set gyro range		*/
	bool ca;	/* calibrate	*/
	bool re;	/* init reset mode		*/
	bool du;	/* dump registers		*/
	bool ne;	/* network start		*/
	bool ts;	/* set time to run 		*/
	bool gf;	/* set g' force magnitude	*/ 
	bool is;	/* set g' force magnitude	*/ 
	unsigned int timesec;	/* samplerate value		*/
	unsigned int steps;	/* integration steps 		*/
	unsigned int dlpfv;	/* samplerate value		*/
	unsigned int clksel;	/* clksel value			*/
	unsigned int smprt;	/* samplerate value		*/
	unsigned int a_ran;	/* accel range value		*/
	unsigned int g_ran;	/* gyro range value		*/
	unsigned int c_sam;	/* cal samples value		*/
	long double gforce;	/* gforce value */
	char d_fln[256];/* dump filename		*/
	char n_hos[256];/* network host			*/
	char n_por[16];	/* network port			*/
};

int mpu_opt_get(struct option *lopts, int argc, char **argv, struct mpu_opt *mopts);
int mpu_opt_set(struct mpu_dev *dev, struct mpu_opt *mopts);
void mpu_opt_pri(struct mpu_opt *mopts);

#endif /* _DEMO_OPT_H_ */
#ifdef __cplusplus
	}
#endif
