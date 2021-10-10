#include <assert.h>
#include <inttypes.h>
#include <getopt.h>
#include "dev_mpu.h"
#include "dev_mpu_opt.h"
#include "dev_mpu_flt.h"
#include "imu.h"
#include "imu_get_data_mpu6050.h"

//#define MDEBUG printf("%3d %-12s\n", __LINE__, __func__);

void mpu_print_all(struct mpu_dev *dev, char *msg, char *buf);
void mpu_print_data  (struct mpu_dev *dev, int times);
void mpu_print_bias  (struct mpu_dev *dev);

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

	//char *msg = malloc(sizeof(char)*MPU_MAXLINE);
	//char *buf = malloc(sizeof(char)*MPU_MAXLINE);

	/* The world frame will be inertial */
	struct frame *fr_world = (struct frame *)calloc(1, sizeof(struct frame)); 
	fr_world->type = frNED;
	fr_world->ref  = NULL;

	/* The body frame tracked relative to world */
	struct frame *fr_body = (struct frame *)calloc(1, sizeof(struct frame)); 
	fr_body->type = frFRD;
	fr_body->ref  = fr_world;

	struct IMU *imu_dev = (struct IMU *)calloc(1, sizeof(struct IMU));
	imu_dev->dev = dev;			/* the driver instance	*/
	imu_dev->fr  = fr_body;			/* the frame of data	*/

	while(1) {
		imu_get_data_mpu6050(imu_dev);
		imu_estimate(imu_dev);
		//imu_print_Phi(imu_dev);
		//mpu_ctl_fifo_data(dev);
		//mpu_ctl_fix_axis(dev);
		//mpu_flt_com_update(flt);
		//strcat(msg,"\r");
		//mpu_print_all(dev, msg, buf);
		//mpu_ang_pri(flt->anf, msg, buf);
		//strcat(msg,"\n");

		//if (!mopts->quiet) {
		//	printf("%s", msg);
		//}

		//if (mopts->ne && (sfd >= 0)) {
		//	mpu_socket_sendmsg(&sfd, msg);
		//}

		//sprintf(msg,"%s", "");
	}

	//free(msg);
	//free(buf);
	return 0;
}
void mpu_print_all(struct mpu_dev *dev, char *msg, char *buf)
{
	sprintf(buf, "%12llu ", dev->samples);
	strcat(msg, buf);

	sprintf(buf, "Hz:%3.0lf ", dev->sr);
	strcat(msg,buf);

	if(dev->cfg->temp_fifo_en) {
		sprintf(buf,  "T:%+.1lf",*(dev->t));
		strcat(msg,buf);
	}
	if(dev->cfg->accel_fifo_en) {
		sprintf(buf, " |A|=%lf ", *(dev->AM) );
		strcat(msg,buf);
		sprintf(buf, "Ax:%+lf Ay:%+lf Az:%+lf ", *(dev->Ax), *(dev->Ay), *(dev->Az));
		strcat(msg,buf);
	}
	if(dev->cfg->xg_fifo_en) {
		sprintf(buf, "Gx:%+lf ", *(dev->Gx));
		strcat(msg,buf);
	}
	if(dev->cfg->yg_fifo_en) {
		sprintf(buf, "Gy:%+lf ", *(dev->Gy));
		strcat(msg,buf);
	}
	if(dev->cfg->zg_fifo_en) {
		sprintf(buf, "Gz:%+lf ", *(dev->Gz));
		strcat(msg,buf);
	}
}

void mpu_print_data(struct mpu_dev *dev, int times)
{
	for (int i = 0; i < times; i++) {
		printf("%3d Hz:%#+3.0lf abdw=%#+4.2lf gbdw=%#+4.2lf  afr=%#+4.2lf  xfr=%#+4.2lf ", i, dev->sr, dev->abdw, dev->gbdw, dev->afr, dev->gfr);
		for (int j = 1; j < (1 + dev->dat->raw[0]); j++) {
			printf(" D[%d]=%+3.4lf", j, dev->dat->dat[j][0]);
		}
		printf("\n");
	}
}


void mpu_print_bias(struct mpu_dev *dev)
{
	int16_t Ax_o = 0;
	int16_t Ay_o = 0;
	int16_t Az_o = 0;
	int16_t Gx_o = 0;
	int16_t Gy_o = 0;
	int16_t Gz_o = 0;
	long double Ax_O = 0;
	long double Ay_O = 0;
	long double Az_O = 0;
	long double Gx_O = 0;
	long double Gy_O = 0;
	long double Gz_O = 0;
	mpu_read_data(dev, XA_OFFS_USRH, &Ax_o);
	mpu_read_data(dev, YA_OFFS_USRH, &Ay_o);
	mpu_read_data(dev, ZA_OFFS_USRH, &Az_o);
	mpu_read_data(dev, XG_OFFS_USRH, &Gx_o);
	mpu_read_data(dev, YG_OFFS_USRH, &Gy_o);
	mpu_read_data(dev, ZG_OFFS_USRH, &Gz_o);
	Ax_O = (long double)Ax_o /(long double)dev->albs;
	Ay_O = (long double)Ay_o /(long double)dev->albs;
	Az_O = (long double)Az_o /(long double)dev->albs;
	Gx_O = (long double)Gx_o /(long double)dev->glbs;
	Gy_O = (long double)Gy_o /(long double)dev->glbs;
	Gz_O = (long double)Gz_o /(long double)dev->glbs;
	printf(" Ax_O=%Lf ", Ax_O); 
	printf(" Ay_O=%Lf ", Ay_O);
	printf(" Az_O=%Lf ", Az_O);
	printf(" Gx_O=%Lf ", Gx_O);
	printf(" Gy_O=%Lf ", Gy_O);
	printf(" Gz_O=%Lf ", Gz_O);
}

