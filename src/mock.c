#include <assert.h>
#include <inttypes.h>
#include <getopt.h>
#include "dev_mpu.h"
#include "dev_mpu_print.h"
#include "dev_mpu_opt.h"
#include "dev_mpu_flt.h"


struct option lopts[] = {
	{"quiet",	no_argument,		0, 0},
	{"reset",	no_argument,		0, 0},
	{"clksel",	required_argument,	0, 0},
	{"dlpf",	required_argument,	0, 0},
	{"rate",	required_argument,	0, 0},
	{"arange",	required_argument,	0, 0},
	{"grange",	required_argument,	0, 0},
	{"dump",	required_argument,	0, 0},
	{"calibrate",	required_argument,	0, 0},
	{"connect",	required_argument,	0, 0},
	{0,		0,		   	0, 0},
};

int main(int argc, char *argv[])
{
	struct mpu_opt *mopts = (struct mpu_opt *)calloc(1, sizeof(struct mpu_opt));
	mpu_opt_get(lopts, argc, argv, mopts);

	struct mpu_dev *dev = NULL;
	if (mopts->re)
		mpu_init("/dev/i2c-1", MPU6050_ADDR, &dev, MPU_MODE_RESET);
	else
		mpu_init("/dev/i2c-1", MPU6050_ADDR, &dev, MPU_MODE_RESTORE);

	assert(dev != NULL);

	mpu_opt_set(dev, mopts);

	struct mpu_flt_dat *flt = NULL;
	mpu_flt_com_init(dev, &flt);
	assert(flt != NULL);

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
		mpu_ctl_fifo_data(dev);
		mpu_ctl_fix_axis(dev);
		mpu_flt_com_update(flt);
		mpu_print_all(dev, msg, buf);
		mpu_ang_pri(flt->anf, msg, buf);
		strcat(msg,"\n");

		if (!mopts->quiet) {
			printf("%s", msg);
		}

		if (mopts->ne && (sfd >= 0)) {
			mpu_socket_sendmsg(&sfd, msg);
		}

		sprintf(msg,"%s", "");
	}

	free(msg);
	free(buf);
	return 0;
}
