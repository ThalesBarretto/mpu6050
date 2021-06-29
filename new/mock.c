#include "dev_mpu.h"
#include "dev_mpu_flt.h"
#include <assert.h>
#include <inttypes.h>

#define MDEBUG printf("%3d %-12s\n", __LINE__, __func__);

void mpu_print_data  (struct mpu_dev *dev, int times);
void mpu_print_all   (struct mpu_dev *dev);
void mpu_print_bias  (struct mpu_dev *dev);

int main(int argc, char *argv[])
{
//	char *host = argv[1];
//	char *port = argv[2];
//	int sfd;
//	if (mpu_socket_connect(&sfd, host, port) < 0) {
//		fprintf(stderr, "socket client error\n");
//	} else {
//		fprintf(stderr,"connected sfd:%d\n", sfd);
//	}
//	if (mpu_socket_sendmsg(&sfd, "SUCESS") < 0)
//		fprintf(stderr," send FAILURE\n");
//
//
//	char *msg = malloc(sizeof(char)*MPU_SOCK_BUFFSIZ);
//	char *buf = malloc(sizeof(char)*MPU_SOCK_BUFFSIZ);
//
//	mpu_socket_makemsg(dev, msg,  buf);
//	mpu_socket_sendmsg(&sfd, buf);
//	printf("The size of mpu6050:%d\n", sizeof(struct mpu6050));

	struct mpu_dev *dev = NULL;
	//mpu_init("/dev/i2c-1", MPU6050_ADDR, &dev, MPU_MODE_RESET);
	mpu_init("/dev/i2c-1", MPU6050_ADDR, &dev, MPU_MODE_RESTORE);
	assert(dev != NULL);

	struct mpu_flt_dat *flt = NULL;
	mpu_flt_com_init(dev, &flt);
	assert(flt != NULL);


	//mpu_ctl_calibration(dev);	
	//printf("Saving parameters\n");
	//mpu_dev_parameters_restore_bin(fn, dev);
	mpu_ctl_samplerate(dev, 200);	
//	mpu_ctl_clocksource(dev, CLKSEL_1);	
//	mpu_ctl_dlpf(dev,5);
//	mpu_ctl_accel_range(dev, 2);
//	mpu_ctl_gyro_range(dev,250);

	while(1) {
		mpu_ctl_fifo_data(dev);
		mpu_ctl_fix_axis(dev);
		mpu_flt_com_update(flt);
		if ((dev->samples % 1) ==0) {
			printf("%12llu ", dev->samples);
			mpu_print_all(dev);
			mpu_ang_pri(flt->anf);
			printf("\n");
		}
	}

	return 0;
}
void mpu_print_all(struct mpu_dev *dev)
{
	printf("%3.0lf Hz ", dev->sr);
	if(dev->cfg->temp_fifo_en) {
		printf( "T:%+.1lf°C ",*(dev->t));
	}
	if(dev->cfg->accel_fifo_en) {
		printf("|A|=%lf ", *(dev->AM) );
		printf("Ax:%+lf Ay:%+lf Az:%+lf ", *(dev->Ax), *(dev->Ay), *(dev->Az));
	}
	if(dev->cfg->xg_fifo_en) {
		printf("Gx:%+lf ", *(dev->Gx));
	}
	if(dev->cfg->yg_fifo_en) {
		printf("Gy:%+lf ", *(dev->Gy));
	}
	if(dev->cfg->zg_fifo_en) {
		printf("Gz:%+lf ", *(dev->Gz));
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
