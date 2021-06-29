#include "dev_mpu.h"
#include <assert.h>
#include <inttypes.h>

#define MDEBUG printf("%3d %-12s\n", __LINE__, __func__);

#define PI (3.1415926535897932384L)
#define r2d (180.0L / PI) 
struct mpu_ang {
	struct mpu_dev *dev;
	int isd;		/* 0 for radians, 1 for degrees 		*/
	long double  ean[3];	/* euler angles (phi, theta, psi) 		*/
	long double  axa[4];	/* axis(x,y,z,theta) for (phi, theta, psi)	*/
	long double  qua[4];	/* quaternion for (phi, theta, psi		*/
	long double  rtm[9];	/* rotation matrix for (phi, theta, psi)	*/
};

struct mpu_flt_dat {
	/* device from which data is taken */
	struct mpu_dev *dev;
	struct mpu_ang *anf;	/* filtered angles */
	struct mpu_ang *ana;	/* accel angle */
	struct mpu_ang *ang;	/* gyro angle */
	struct mpu_ang *anm;	/* mag angle */
	int isd;		/* 0 for radians, 1 for degrees 		*/
	long double  ean[3];	/* euler angles (phi, theta, psi) 		*/
	long double  axa[4];	/* axis(x,y,z,theta) for (phi, theta, psi)	*/
	long double  qua[4];	/* quaternion for (phi, theta, psi		*/
	long double  rtm[9];	/* rotation matrix for (phi, theta, psi)	*/

	long double  ear[3];	/* euler angles rates  (phi, theta, psi) 	*/
	long double  cpm[9];	/* cross product matrix */
	long double  cp2[9];	/* cross product matrix square */
	long double *phi;   	/* roll angle 		*/
	long double *the;   	/* pitch angle 		*/
	long double *psi;   	/* yaw angle 		*/
	long double *phr;   	/* roll angle rate	*/
	long double *thr;   	/* pitch angle rate	*/
	long double *psr;   	/* yaw angle rate	*/

	/* euler angles rates */
	long double rho;    /* tilt angle 		*/
	long double rhr;    /* tilt angle rate 	*/

	long double gain;
	long double mtr; /* motion treshold acceleration in g's		*/
	long double ant; /* axis acceleration noise treshold		*/ 

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
};

void mpu_ang_init(struct mpu_dev *, struct mpu_ang **ang);
void mpu_ang_acc(struct mpu_ang *);
void mpu_ang_gyr(struct mpu_ang *base, struct mpu_ang *ang);

void mpu_flt_com_update(struct mpu_flt_dat *flt);
void mpu_flt_com_init(struct mpu_dev *dev, struct mpu_flt_dat **flt);
void mpu_flt_com_print_angle (struct mpu_flt_dat *flt);

void mpu_print_data  (struct mpu_dev *dev, int times);
void mpu_print_angle (struct mpu_flt_dat *flt);
void mpu_print_all   (struct mpu_dev *dev);
void mpu_print_bias  (struct mpu_dev *dev);

void mpu_ang_init(struct mpu_dev * dev, struct mpu_ang **ang)
{
	*ang = (struct mpu_ang *)calloc(1, sizeof(struct mpu_ang));
	assert(NULL != *ang);

	(*ang)->dev = dev;
	assert(NULL != (*ang)->dev);

	/* assume degrees */
	(*ang)->isd = 1;
}

/*  ASSUME DEGREES integrate gyro angle change from gyro rates */
void mpu_ang_gyr(struct mpu_ang *base, struct mpu_ang *ang) 
{
	/* assume constant rate */
	ang->ean[0] = (*(ang->dev->Gx) * ang->dev->st);
	ang->ean[1] = (*(ang->dev->Gy) * ang->dev->st);
	ang->ean[2] = (*(ang->dev->Gz) * ang->dev->st);
	/* add to base */
	if ((base != NULL) && (base != 0)) {
		ang->ean[0] += base->ean[0];
		ang->ean[1] += base->ean[1];
		ang->ean[2] += base->ean[2];
	}
}

/* ASSUME DEGREES - Estimate angles from accel */
void mpu_ang_acc(struct mpu_ang *ang)
{
	ang->ean[0] = -r2d * (atan2l(1.L* *(ang->dev->Ay), -1.L * *(ang->dev->Az)));
	ang->ean[1] = -r2d * (atan2l(1.L* *(ang->dev->Ax), -1.L * *(ang->dev->Az)));
	ang->ean[2] = 0;
}

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

	struct mpu_dev *dev = NULL;
	mpu_init("/dev/i2c-1", MPU6050_ADDR, &dev);
	assert(dev != NULL);
	char *fn = "dev_mpu_parameters.log";

	struct mpu_flt_dat *flt = NULL;
	mpu_flt_com_init(dev, &flt);
	assert(flt != NULL);


	mpu_ctl_calibration(dev);	
	mpu_ctl_dlpf(dev,5);
	mpu_ctl_accel_range(dev, 2);
	mpu_ctl_gyro_range(dev,250);

	mpu_dev_parameters_dump(fn, dev);
	//mpu_dev_parameters_restore(fn, dev);
	while(1) {
		mpu_ctl_fifo_data(dev);
		mpu_ctl_fix_axis(dev);
		mpu_flt_com_update(flt);
		if ((dev->samples % 1) ==0) {
			printf("%12llu ", dev->samples);
			mpu_print_all(dev);
			mpu_flt_com_print_angle(flt);
			printf("\n");
		}
	}

	return 0;
}

void mpu_flt_com_update(struct mpu_flt_dat *flt)
{
	/* Estimate angles from accel */
	mpu_ang_acc(flt->ana);
	//flt->Rx_last = -r2d * (atan2l(1.L* *(flt->dev->Ay), -1.L * *(flt->dev->Az)));
	//flt->Ry_last = -r2d * (atan2l(1.L* *(flt->dev->Ax), -1.L * *(flt->dev->Az)));

	/* integrate gyro rates based on last filter data */
	mpu_ang_gyr(flt->anf, flt->ang);
	//flt->Gx_last = flt->dev->ang->phi + (*(flt->dev->Gx) * flt->dev->st);
	//flt->Gy_last = flt->dev->ang->the + (*(flt->dev->Gy) * flt->dev->st);

	/* motion detected, dont trust accelerometer */
	if (fabs( (*(flt->dev->AM) -flt->dev->cal->gra) > flt->mtr) ) {
		flt->anf->ean[0] =  flt->ang->ean[0];
		flt->anf->ean[1] =  flt->ang->ean[1];
		flt->anf->ean[2] =  flt->ang->ean[2];
	} else {
		/* fuse accel and gyro by complementary filter */
		flt->anf->ean[0] = ((1 - flt->gain) * flt->ana->ean[0]) + (flt->gain * flt->ang->ean[0]);
		flt->anf->ean[1] = ((1 - flt->gain) * flt->ana->ean[1]) + (flt->gain * flt->ang->ean[1]);
	}
	/* psi angle come only from gyro */
	flt->ean[2] +=  flt->ang->ean[2];

	flt->rho =  r2d * (atan2l (sqrtl(*(flt->dev->Ax2) + *(flt->dev->Ay2)), -*(flt->dev->Az)));

}

void mpu_flt_com_init(struct mpu_dev *dev, struct mpu_flt_dat **flt)
{
	*flt = (struct mpu_flt_dat *)calloc(1, sizeof(struct mpu_flt_dat));
	assert(NULL != *flt);

	(*flt)->dev = dev;
	assert(NULL != (*flt)->dev);

	(*flt)->anf = (struct mpu_ang *)calloc(1, sizeof(struct mpu_ang));
	assert(NULL != (*flt)->anf);
	(*flt)->ana = (struct mpu_ang *)calloc(1, sizeof(struct mpu_ang));
	assert(NULL != (*flt)->ana);
	(*flt)->ang = (struct mpu_ang *)calloc(1, sizeof(struct mpu_ang));
	assert(NULL != (*flt)->ang);
	(*flt)->anm = (struct mpu_ang *)calloc(1, sizeof(struct mpu_ang));
	assert(NULL != (*flt)->anm);

	mpu_ang_init(dev, &(*flt)->anf); 
	mpu_ang_init(dev, &(*flt)->ana); 
	mpu_ang_init(dev, &(*flt)->ang); 
	mpu_ang_init(dev, &(*flt)->anm); 

	(*flt)->gain = 0.8L;
	(*flt)->mtr = 0.05L;
	(*flt)->ant = 0.002L;
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

void mpu_flt_com_print_angle(struct mpu_flt_dat *flt)
{
	printf("phi:%+.2Lf ", flt->anf->ean[0]);
	printf("the:%+.2Lf ", flt->anf->ean[1]);
	printf("psi:%+.2Lf ", flt->anf->ean[2]);
	printf("rho:%+.2Lf ", flt->rho);
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
	fprintf(dmp, "%-20s %"PRIx8"\n"	,"PRODUCT_ID"		, dev->prod_id		);
	fprintf(dmp, "%-20s %6u\n"	,"CLOCK_SOURCE"		, dev->clksel		);
	fprintf(dmp, "%-20s %6u\n"	,"LOWPASS_FILTER"	, dev->dlpf		);
	fprintf(dmp, "%-20s %6.0lf\n"	,"SAMPLING_RATE"	, dev->sr		);
	fprintf(dmp, "%-20s %6.0lf\n"	,"ACCEL_FULL_RANGE"	, dev->afr		);
	fprintf(dmp, "%-20s %6.0lf\n"	,"GYRO_FULL_RANGE"	, dev->gfr		);
	fprintf(dmp, "%-20s %"PRId16"\n","xa_cust"		, dev->cal->xa_cust	);
	fprintf(dmp, "%-20s %"PRId16"\n","ya_cust"		, dev->cal->ya_cust	);
	fprintf(dmp, "%-20s %"PRId16"\n","za_cust"		, dev->cal->za_cust	);
	fprintf(dmp, "%-20s %"PRId16"\n","xg_cust"		, dev->cal->xg_cust	);
	fprintf(dmp, "%-20s %"PRId16"\n","yg_cust"		, dev->cal->yg_cust	);
	fprintf(dmp, "%-20s %"PRId16"\n","zg_cust"		, dev->cal->zg_cust	);
	fprintf(dmp, "%-20s %4.16Lf\n",  "xa_bias"		, dev->cal->xa_bias	);
	fprintf(dmp, "%-20s %4.16Lf\n",  "ya_bias"		, dev->cal->ya_bias	);
	fprintf(dmp, "%-20s %4.16Lf\n",  "za_bias"		, dev->cal->za_bias	);
	fprintf(dmp, "%-20s %4.16Lf\n",  "xg_bias"		, dev->cal->xg_bias	);
	fprintf(dmp, "%-20s %4.16Lf\n",  "yg_bias"		, dev->cal->yg_bias	);
	fprintf(dmp, "%-20s %4.16Lf\n",  "zg_bias"		, dev->cal->zg_bias	);
	fprintf(dmp, "%-20s %4.16Lf\n",  "AM_bias"		, dev->cal->AM_bias	);
	fprintf(dmp, "%-20s %4.16Lf\n",  "GM_bias"		, dev->cal->GM_bias	);

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
	loffset = ftell(fp);

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
	if (strcmp(buf, "PRODUCT_ID"		) == 0) { fscanf(fp, "%"PRIx8,	&prod_id);	}
	if (strcmp(buf, "CLOCK_SOURCE"		) == 0) { fscanf(fp, "%u",	&clksel);	}
	if (strcmp(buf, "LOWPASS_FILTER"	) == 0) { fscanf(fp, "%u",	&dlpf);		}
	if (strcmp(buf, "SAMPLING_RATE"		) == 0) { fscanf(fp, "%lf",	&sr);	 	}
	if (strcmp(buf, "ACCEL_FULL_RANGE"	) == 0) { fscanf(fp, "%lf",	&afr);		}
	if (strcmp(buf, "GYRO_FULL_RANGE"	) == 0) { fscanf(fp, "%lf",	&gfr);		}
	if (strcmp(buf, "xa_cust"		) == 0) { fscanf(fp, "%"PRId16,	&xa_cust);	}
	if (strcmp(buf, "ya_cust"		) == 0) { fscanf(fp, "%"PRId16,	&ya_cust);	}
	if (strcmp(buf, "za_cust"		) == 0) { fscanf(fp, "%"PRId16,	&za_cust);	}
	if (strcmp(buf, "xg_cust"		) == 0) { fscanf(fp, "%"PRId16,	&xg_cust);	}
	if (strcmp(buf, "yg_cust"		) == 0) { fscanf(fp, "%"PRId16,	&yg_cust);	}
	if (strcmp(buf, "zg_cust"		) == 0) { fscanf(fp, "%"PRId16,	&zg_cust);	}
	if (strcmp(buf, "xa_bias"		) == 0) { fscanf(fp, "%Lf", 	&bkp->xa_bias);	}
	if (strcmp(buf, "ya_bias"		) == 0) { fscanf(fp, "%Lf", 	&bkp->ya_bias);	}
	if (strcmp(buf, "za_bias"		) == 0) { fscanf(fp, "%Lf", 	&bkp->za_bias);	}
	if (strcmp(buf, "xg_bias"		) == 0) { fscanf(fp, "%Lf", 	&bkp->xg_bias);	}
	if (strcmp(buf, "yg_bias"		) == 0) { fscanf(fp, "%Lf", 	&bkp->yg_bias);	}
	if (strcmp(buf, "zg_bias"		) == 0) { fscanf(fp, "%Lf", 	&bkp->zg_bias);	}
	if (strcmp(buf, "AM_bias"		) == 0) { fscanf(fp, "%Lf", 	&bkp->AM_bias);	}
	if (strcmp(buf, "GM_bias"		) == 0) { fscanf(fp, "%Lf", 	&bkp->GM_bias);	}
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

	fprintf(stdout, "MPU6050\n");
	fprintf(stdout, "%-20s %"PRIx8"\n",	"PRODUCT ID",		prod_id		);
	fprintf(stdout, "%-20s %6d\n"	,	"CLOCK_SOURCE",		clksel		);
	fprintf(stdout, "%-20s %6d\n"	,	"LOWPASS_FILTER",	dlpf		);
	fprintf(stdout, "%-20s %6.0lf\n",	"SAMPLING_RATE"	, 	sr		);
	fprintf(stdout, "%-20s %6.0lf\n",	"ACCEL_FULL_RANGE",	afr		);
	fprintf(stdout, "%-20s %6.0lf\n",	"GYRO_FULL_RANGE",	gfr		);
	fprintf(stdout, "%-20s %"PRId16"\n",	"xa_cust",		bkp->xa_cust	);
	fprintf(stdout, "%-20s %"PRId16"\n",	"ya_cust",		bkp->ya_cust	);
	fprintf(stdout, "%-20s %"PRId16"\n",	"za_cust",		bkp->za_cust	);
	fprintf(stdout, "%-20s %"PRId16"\n",	"xg_cust",		bkp->xg_cust	);
	fprintf(stdout, "%-20s %"PRId16"\n",	"yg_cust",		bkp->yg_cust	);
	fprintf(stdout, "%-20s %"PRId16"\n",	"zg_cust",		bkp->zg_cust	);
	fprintf(stdout, "%-20s %4.16Lf\n",	"xa_bias",		bkp->xa_bias	);
	fprintf(stdout, "%-20s %4.16Lf\n",	"ya_bias",		bkp->ya_bias	);
	fprintf(stdout, "%-20s %4.16Lf\n",	"za_bias",		bkp->za_bias	);
	fprintf(stdout, "%-20s %4.16Lf\n",	"xg_bias",		bkp->xg_bias	);
	fprintf(stdout, "%-20s %4.16Lf\n",	"yg_bias",		bkp->yg_bias	);
	fprintf(stdout, "%-20s %4.16Lf\n",	"zg_bias",		bkp->zg_bias	);
	fprintf(stdout, "%-20s %4.16Lf\n",	"AM_bias",		bkp->AM_bias	);
	fprintf(stdout, "%-20s %4.16Lf\n",	"GM_bias",		bkp->GM_bias	);

	fclose(fp);

}

