// SPDX-License-Identifier: MIT
/* Copyright (C) 2021 Thales Antunes de Oliveira Barretto */
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <getopt.h>
#include <libmpu6050/mpu6050_core.h>
#include "demo_opt.h"
#include "demo_socket.h"
#include "filter.h"
#include "state.h"

#define debug_exit(X) fprintf(stderr, "DEBUG_EXIT:\nline:%i func:%s\n",__LINE__, __func__ ); exit((X));
#define MPU_MAXLINE 1024

void snprint_data (struct mpu_dev *dev, char *msg, char *buf);
void snprint_angle(struct mpu_ang *ang, char *msg, char *buf);

struct option lopts[] = {
	{"quiet",	no_argument,		0, 0},
	{"reset",	no_argument,		0, 0},
	{"calibrate",	no_argument,		0, 0},
	{"timesec",	required_argument,	0, 0},
	{"clksel",	required_argument,	0, 0},
	{"dlpf",	required_argument,	0, 0},
	{"rate",	required_argument,	0, 0},
	{"arange",	required_argument,	0, 0},
	{"grange",	required_argument,	0, 0},
	{"dump",	required_argument,	0, 0},
	{"connect",	required_argument,	0, 0},
	{0,		0,		   	0, 0},
};

int main(int argc, char *argv[])
{

	struct mpu_opt *mopts = (struct mpu_opt *)calloc(1, sizeof(struct mpu_opt));
	mpu_opt_get(lopts, argc, argv, mopts);


	struct mpu_dev *dev = NULL;
	if (mopts->re)
		mpu_init("/dev/i2c-1", &dev, MPU6050_RESET);
	else
		mpu_init("/dev/i2c-1", &dev, MPU6050_RESTORE);

	if (NULL == dev) {
		fprintf(stderr,"Unable to create device, aborting\n");
		abort();
	}

	mpu_opt_set(dev, mopts);
	/* initialize a series of states */
	state_t State = state_zero();
	series_t *Series = series_new(1);
	/* employ 120 seconds of samples */
	/* Use Trapezoidal method, in 100 steps */
	integration_t trapez100 = {
		.method = RK_TRAPEZ,
		.steps = 100
	};
	position_t origin = { .p={ 0, 0, 0 }};
	position_t cm_pos = { .p={ 0, 0, 0 }};
	imu_t Imu = {
		.description = "MPU-6050",
		.dat = { 0,0,0,0,0,0 },
		.cfg = { 0, 0, 0 },
		.relpos = {
			.reference = &cm_pos,
			.pos  = { .p1=0, .p2=0, .p3=0 },
			.att  = { .phi= M_PI, .theta=0, .psi=0 }
		}
	};

	printf("Accel_range	=%lf\n", dev->afr);
	printf("Gyro_range	=%lf\n", dev->gfr);
	printf("Fs		=%lf\n", dev->sr );
	Imu.cfg.Accel_range	= dev->afr;
	Imu.cfg.Gyro_range	= dev->gfr;
	Imu.cfg.Fs		= dev->sr;
	series_init(Series, "Series testing", &Imu, &trapez100, (Imu.cfg.Fs * mopts->timesec));
	printf("Run for %u seconds\n", mopts->timesec);
	printf("Run for %zu size\n", Series->size);
	exit(EXIT_SUCCESS);

	struct mpu_flt_dat *flt = NULL;
	filter_init(dev, &flt);
	if (NULL == flt) {
		fprintf(stderr,"Unable to create filter, aborting\n");
		abort();
	}

	int sfd = -1;
	if (mopts->ne) {
		if (mpu_socket_connect(&sfd, mopts->n_hos, mopts->n_por) < 0) {
			fprintf(stderr, "socket client error\n");
		} else {
			fprintf(stderr,"connected sfd:%d\n", sfd);
		}
	}

	char *msg = calloc(1, sizeof(char)*MPU_MAXLINE);
	char *buf = calloc(1, sizeof(char)*MPU_MAXLINE);

	while(1) {
		mpu_get_data(dev);
		filter_update(flt);

		snprint_data(dev, msg, buf);
		snprint_angle(flt->anf, msg, buf);
		strcat(msg,"\n");

		if (!mopts->quiet)
			printf("%s", msg);

		if (mopts->ne && (sfd >= 0))
			mpu_socket_sendmsg(&sfd, msg);

		sprintf(msg,"%s", "");
	}
	filter_destroy(flt);
	mpu_destroy(dev);
	free(msg);
	free(buf);
	return 0;
}
