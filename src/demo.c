#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <getopt.h>
#include <libmpu6050/mpu6050_core.h>
#include "demo_opt.h"
#include "demo_socket.h"
#include "filter.h"

#define MPU_MAXLINE 1024

void snprint_data (struct mpu_dev *dev, char *msg, char *buf);
void snprint_angle(struct mpu_ang *ang, char *msg, char *buf);

struct option lopts[] = {
	{"quiet",	no_argument,		0, 0},
	{"reset",	no_argument,		0, 0},
	{"calibrate",	no_argument,		0, 0},
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

	char *msg = malloc(sizeof(char)*MPU_MAXLINE);
	char *buf = malloc(sizeof(char)*MPU_MAXLINE);

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
