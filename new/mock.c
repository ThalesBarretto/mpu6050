#include "dev_mpu.h"
#include <assert.h>

#define MDEBUG printf("%3d %-12s\n", __LINE__, __func__);

struct mpu_flt_compl {
	struct mpu_dev *dev;
	long double gain;

	/* angles taken from accelerometer */
	long double AM_last;
	long double GM_last;
	long double xa_last;
	long double ya_last;
	long double za_last;
	long double Rx_last;
	long double Ry_last;
	long double Rz_last;
	long double Ro_last;
	long double Gx_last;
	long double Gy_last;
	long double Gz_last;

	/* previous values */
	long double AM_prev;
	long double GM_prev;
	long double xa_prev;
	long double ya_prev;
	long double za_prev;
	long double Gx_prev;
	long double Gy_prev;
	long double Gz_prev;
	long double Rx_prev;
	long double Ry_prev;
	long double Rz_prev;
	long double Rx_rate;
	long double Ry_rate;
	long double Rz_rate;

	long double Rx_final;
	long double Ry_final;
	long double Rz_final;

};

int mpu_flt_compl_update(struct mpu_flt_compl *flt);
int mpu_flt_compl_init(struct mpu_dev *dev, struct mpu_flt_compl **flt);
void mpu_flt_compl_print_angle (struct mpu_flt_compl *flt);

void mpu_print_data  (struct mpu_dev *dev, int times);
void mpu_print_angle (struct mpu_flt_compl *flt);
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

	struct mpu_dev *dev = NULL;
	mpu_init("/dev/i2c-1", MPU6050_ADDR, &dev);
	assert(dev != NULL);

	struct mpu_flt_compl *flt = NULL;
	mpu_flt_compl_init(dev, &flt);
	assert(flt != NULL);

	mpu_ctl_calibration(dev);	
	mpu_ctl_dlpf(dev,5);
	mpu_ctl_accel_range(dev, 2);
	mpu_ctl_gyro_range(dev,250);

	while(1) {
		mpu_ctl_fifo_data(dev);
		mpu_ctl_fix_axis(dev);
		printf("%12llu ", dev->samples);
		mpu_flt_compl_update(flt);
		mpu_print_all(dev);
		mpu_flt_compl_print_angle(flt);
		printf("\n");
	}

	return 0;
}

int mpu_flt_compl_update(struct mpu_flt_compl *flt)
{
#define PI (3.1415926535897932384L)
#define r2d (180.0L / PI) 
	/* assume stationary device, angles relative to fixed reference frame */
	flt->Rx_last =  -r2d * (atan2l((long double)*(flt->dev->Ay),-1.L * (long double)*(flt->dev->Az)));
	flt->Ry_last =   r2d * (atan2l((long double)*(flt->dev->Ax),-1.L * (long double)*(flt->dev->Az)));
	//flt->Gx_last = flt->dev->ang->Rx_earth + (*(flt->dev->Gx) * (1/(long double)flt->dev->sr));
	//flt->Gy_last = flt->dev->ang->Ry_earth + (*(flt->dev->Gy) * (1/(long double)flt->dev->sr));
	flt->Gx_last = flt->dev->ang->Rx_earth + (*(flt->dev->Gx) * flt->dev->st);
	flt->Gy_last = flt->dev->ang->Ry_earth + (*(flt->dev->Gy) * flt->dev->st);
	//flt->dev->ang->Rz_earth +=  ((long double)*(flt->dev->Gz) * (long double)flt->dev->st);
	flt->dev->ang->Ro_earth =  r2d * (atan2l (sqrtl(*(flt->dev->Ax2) + *(flt->dev->Ay2)), -*(flt->dev->Az)));


	flt->dev->ang->Rx_earth = ((1 - flt->gain) * flt->Rx_last) + (flt->gain * flt->Gx_last);
	flt->dev->ang->Ry_earth = ((1 - flt->gain) * flt->Ry_last) + (flt->gain * flt->Gy_last);
	//flt->Rz_final = ((1-flt->gain)*flt->Rz_last) + (flt->gain * (flt->Rz_final+(*(flt->dev->Gz) * flt->dev->st)));
	return 0;

#undef PI
#undef r2d
}

int mpu_flt_compl_init(struct mpu_dev *dev, struct mpu_flt_compl **flt)
{
	*flt = (struct mpu_flt_compl *)calloc(1, sizeof(struct mpu_flt_compl));
	assert(NULL != *flt);

	(*flt)->dev = dev;
	assert(NULL != (*flt)->dev);

	(*flt)->gain = 0.8L;
	(*flt)->Rx_last = 0;
	(*flt)->Ry_last = 0;
	(*flt)->Rz_last = 0;
	(*flt)->Ro_last = 0;
	(*flt)->Rx_prev = 0;
	(*flt)->Ry_prev = 0;
	(*flt)->Rz_prev = 0;
	(*flt)->Rx_rate = 0;
	(*flt)->Ry_rate = 0;
	(*flt)->Rz_rate = 0;
	(*flt)->Rx_final = 0;
	(*flt)->Ry_final = 0;
	(*flt)->Rz_final = 0;
	(*flt)->Gx_prev = 0;
	(*flt)->Gy_prev = 0;
	(*flt)->Gz_prev = 0;
	return 0;
}

void mpu_print_all(struct mpu_dev *dev)
{
	//printf("%3.0lf Hz ", dev->sr);
	if(dev->cfg->temp_fifo_en) {
		printf( "T:%+.1lf°C ",*(dev->t));
	}
	if(dev->cfg->accel_fifo_en) {
		printf("|A|=%lf ", fabs(*(dev->AM) -1));
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

void mpu_flt_compl_print_angle(struct mpu_flt_compl *flt)
{
//	printf("Rx_acc:%+.1Lf ", flt->Rx_acc);
//	printf("Ry_acc:%+.1Lf ", flt->Ry_acc);
//	printf("Gx_acc:%+.1Lf ", flt->Gx_acc);
//	printf("Gy_acc:%+.1Lf ", flt->Gy_acc);
//	printf("Gz_acc:%+.1Lf ", flt->Gz_acc);
	printf("Rx:%+.2Lf ", flt->dev->ang->Rx_earth);
	printf("Ry:%+.2Lf ", flt->dev->ang->Ry_earth);
	printf("Ro:%+.2Lf ", flt->dev->ang->Ro_earth);
	//printf("Rx:%+.1Lf ", flt->Rx_final_acc);
	//printf("Ry:%+.1Lf ", flt->Ry_final_acc);

//	printf("Rx_rate:%+.1Lf ", flt->Rx_rate);
//	printf("Ry_rate:%+.1Lf ", flt->Ry_rate);
//	printf("Gx_rate:%+.1lf ", *(flt->dev->Gx));
//	printf("Gy_rate:%+.1lf ", *(flt->dev->Gy));
//
//	printf("Rx_final:%+.1Lf ", flt->Rx_final);
//	printf("Ry_final:%+.1Lf ", flt->Ry_final);

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

