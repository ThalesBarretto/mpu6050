#include "dev_mpu.h"
#include "dev_mpu_flt.h"
#include <assert.h>
#include <inttypes.h>

#define MDEBUG printf("%3d %-12s\n", __LINE__, __func__);

void mpu_print_data  (struct mpu_dev *dev, int times);
void mpu_print_all   (struct mpu_dev *dev);
void mpu_print_bias  (struct mpu_dev *dev);
void mpu_dev_parameters_dump(char *fn, struct mpu_dev *dev);
void mpu_dev_parameters_restore(char *fn, struct mpu_dev *dev);

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
	mpu_init("/dev/i2c-1", MPU6050_ADDR, &dev);
	assert(dev != NULL);
	char *fn = "dev_mpu_parameters.log";

	struct mpu_flt_dat *flt = NULL;
	mpu_flt_com_init(dev, &flt);
	assert(flt != NULL);


//	mpu_ctl_calibration(dev);	
	//mpu_ctl_clocksource(dev, CLKSEL_1);	
//	mpu_ctl_dlpf(dev,5);
//	mpu_ctl_accel_range(dev, 2);
//	mpu_ctl_gyro_range(dev,250);

	mpu_dev_parameters_dump(fn, dev);
	//mpu_dev_parameters_restore(fn, dev);
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
void mpu_dev_parameters_dump(char *fn, struct mpu_dev *dev)
{
	FILE *dmp;
	dmp = fopen(fn, "w+");
	fprintf(dmp, "MPU6050\n");
	fprintf(dmp, "%s %u\n"	,"PRODUCT_ID"		, dev->prod_id		);
	fprintf(dmp, "%s %u\n"	,"CLOCK_SOURCE"		, dev->clksel		);
	fprintf(dmp, "%s %u\n"	,"LOWPASS_FILTER"	, dev->dlpf		);
	fprintf(dmp, "%s %lf\n"	,"SAMPLING_RATE"	, dev->sr		);
	fprintf(dmp, "%s %lf\n"	,"ACCEL_FULL_RANGE"	, dev->afr		);
	fprintf(dmp, "%s %lf\n"	,"GYRO_FULL_RANGE"	, dev->gfr		);
	fprintf(dmp, "%s %d\n"	,"xa_cust"		, dev->cal->xa_cust	);
	fprintf(dmp, "%s %d\n"	,"ya_cust"		, dev->cal->ya_cust	);
	fprintf(dmp, "%s %d\n"	,"za_cust"		, dev->cal->za_cust	);
	fprintf(dmp, "%s %d\n"	,"xg_cust"		, dev->cal->xg_cust	);
	fprintf(dmp, "%s %d\n"	,"yg_cust"		, dev->cal->yg_cust	);
	fprintf(dmp, "%s %d\n"	,"zg_cust"		, dev->cal->zg_cust	);
	fprintf(dmp, "%s %Lf\n"	,"xa_bias"		, dev->cal->xa_bias	);
	fprintf(dmp, "%s %Lf\n"	,"ya_bias"		, dev->cal->ya_bias	);
	fprintf(dmp, "%s %Lf\n"	,"za_bias"		, dev->cal->za_bias	);
	fprintf(dmp, "%s %Lf\n"	,"xg_bias"		, dev->cal->xg_bias	);
	fprintf(dmp, "%s %Lf\n"	,"yg_bias"		, dev->cal->yg_bias	);
	fprintf(dmp, "%s %Lf\n"	,"zg_bias"		, dev->cal->zg_bias	);
	fprintf(dmp, "%s %Lf\n"	,"AM_bias"		, dev->cal->AM_bias	);
	fprintf(dmp, "%s %Lf\n"	,"GM_bias"		, dev->cal->GM_bias	);

	fclose(dmp);
}

void mpu_dev_parameters_restore(char *fn, struct mpu_dev *dev)
{
#define MPU_MAXLINE 1024 
	FILE * fp;
	if ( (fp = fopen(fn, "r")) == NULL) {
		fprintf(stderr, "Unable to open file \"%s\"\n", fn);
		exit(EXIT_FAILURE);
	}

	char *buf = (char *)calloc(MPU_MAXLINE, sizeof(char));

	long loffset = 0L;
	while (!(feof(fp) || ferror(fp))) {
		fscanf(fp, "%s", buf);
		if (strcmp(buf, "MPU6050") == 0) { break;}
	}
	if ((loffset = ftell(fp)) == -1L) {
		fprintf(stderr, "Couldn't find line offset\n");
	}

	unsigned int prod_id;
	unsigned int clksel;
	unsigned int dlpf;
	double sr;
	double afr;
	double gfr;
	int xa_cust;
	int ya_cust;
	int za_cust;
	int xg_cust;
	int yg_cust;
	int zg_cust;
	struct mpu_cal *bkp = calloc(1, sizeof(struct mpu_cal));
	do {
	fscanf(fp, "%s", buf);
	if (0 == strcmp(buf, "PRODUCT_ID"	)) { fscanf(fp, "%u",	&prod_id);	}
	if (0 == strcmp(buf, "CLOCK_SOURCE"	)) { fscanf(fp, "%u",	&clksel);	}
	if (0 == strcmp(buf, "LOWPASS_FILTER"	)) { fscanf(fp, "%u",	&dlpf);		}
	if (0 == strcmp(buf, "SAMPLING_RATE"	)) { fscanf(fp, "%lf",	&sr);	 	}
	if (0 == strcmp(buf, "ACCEL_FULL_RANGE"	)) { fscanf(fp, "%lf",	&afr);		}
	if (0 == strcmp(buf, "GYRO_FULL_RANGE"	)) { fscanf(fp, "%lf",	&gfr);		}
	if (0 == strcmp(buf, "xa_cust"		)) { fscanf(fp, "%d",	&xa_cust);	}
	if (0 == strcmp(buf, "ya_cust"		)) { fscanf(fp, "%d",	&ya_cust);	}
	if (0 == strcmp(buf, "za_cust"		)) { fscanf(fp, "%d",	&za_cust);	}
	if (0 == strcmp(buf, "xg_cust"		)) { fscanf(fp, "%d",	&xg_cust);	}
	if (0 == strcmp(buf, "yg_cust"		)) { fscanf(fp, "%d",	&yg_cust);	}
	if (0 == strcmp(buf, "zg_cust"		)) { fscanf(fp, "%d",	&zg_cust);	}
	if (0 == strcmp(buf, "xa_bias"		)) { fscanf(fp, "%Lf", 	&bkp->xa_bias);	}
	if (0 == strcmp(buf, "ya_bias"		)) { fscanf(fp, "%Lf", 	&bkp->ya_bias);	}
	if (0 == strcmp(buf, "za_bias"		)) { fscanf(fp, "%Lf", 	&bkp->za_bias);	}
	if (0 == strcmp(buf, "xg_bias"		)) { fscanf(fp, "%Lf", 	&bkp->xg_bias);	}
	if (0 == strcmp(buf, "yg_bias"		)) { fscanf(fp, "%Lf", 	&bkp->yg_bias);	}
	if (0 == strcmp(buf, "zg_bias"		)) { fscanf(fp, "%Lf", 	&bkp->zg_bias);	}
	if (0 == strcmp(buf, "AM_bias"		)) { fscanf(fp, "%Lf", 	&bkp->AM_bias);	}
	if (0 == strcmp(buf, "GM_bias"		)) { fscanf(fp, "%Lf", 	&bkp->GM_bias);	}
	} while (!(feof(fp) || ferror(fp)));
	bkp->xa_cust = xa_cust;
	bkp->ya_cust = ya_cust;
	bkp->za_cust = za_cust;
	bkp->xg_cust = xg_cust;
	bkp->ya_cust = ya_cust;
	bkp->zg_cust = zg_cust;


	mpu_ctl_clocksource(dev, clksel);
	mpu_ctl_dlpf(dev,dlpf);
	mpu_ctl_samplerate(dev, sr);
	mpu_ctl_accel_range(dev, afr);
	mpu_ctl_gyro_range(dev, gfr);
//	mpu_ctl_calibration_restore(dev, bkp);
	fclose(fp);

}

