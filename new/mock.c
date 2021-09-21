#include "dev_mpu.h"
#include "dev_mpu_flt.h"
#include <assert.h>
#include <inttypes.h>
#include <getopt.h>

#define MDEBUG printf("%3d %-12s\n", __LINE__, __func__);

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
	int clksel;	/* clksel value			*/
	int dlpfv;	/* samplerate value		*/
	int smprt;	/* samplerate value		*/
	int a_ran;	/* accel range value		*/
	int g_ran;	/* gyro range value		*/
	int c_sam;	/* cal samples value		*/
	char d_fln[256];/* dump filename		*/
	char n_hos[256];/* network host			*/
	char n_por[16];	/* network port			*/
};

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


int mpu_opt_get(struct option *lopts, int argc, char **argv, struct mpu_opt *mopts);
int mpu_opt_set(struct mpu_dev *dev, struct mpu_opt *mopts);
void mpu_opt_pri(struct mpu_opt *mopts);
void mpu_print_all(struct mpu_dev *dev, char *msg, char *buf);

void mpu_print_data  (struct mpu_dev *dev, int times);
void mpu_print_bias  (struct mpu_dev *dev);

int main(int argc, char *argv[])
{
	struct mpu_opt *mopts = (struct mpu_opt *)calloc(1, sizeof(struct mpu_opt));
	mpu_opt_get(lopts, argc, argv, mopts);

	struct mpu_dev *dev = NULL;
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
		strcat(msg,"\r");
		mpu_print_all(dev, msg, buf);
		mpu_ang_pri(flt->anf, msg, buf);
		//strcat(msg,"\n");

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

int mpu_opt_get(struct option *lopts, int argc, char **argv, struct mpu_opt *mopts)
{
	int c;
	int i = 0;
	while ((c = getopt_long(argc, argv, "", lopts, &i)) != -1) {
		switch(c) {
		case 0:
			if (strcmp(lopts[i].name, "clksel"	) == 0) { mopts->cs = true; sscanf(optarg, "%d", &mopts->clksel); break;}
			if (strcmp(lopts[i].name, "dlpf"	) == 0) { mopts->dl = true; sscanf(optarg, "%d", &mopts->dlpfv); break;}
			if (strcmp(lopts[i].name, "rate"	) == 0) { mopts->sr = true; sscanf(optarg, "%d", &mopts->smprt); break;}
			if (strcmp(lopts[i].name, "arange"	) == 0) { mopts->ar = true; sscanf(optarg, "%d", &mopts->a_ran); break;}
			if (strcmp(lopts[i].name, "grange"	) == 0) { mopts->gr = true; sscanf(optarg, "%d", &mopts->g_ran); break;}
			if (strcmp(lopts[i].name, "dump"	) == 0) { mopts->du = true; sscanf(optarg, "%s",  mopts->d_fln); break;}
			if (strcmp(lopts[i].name, "calibrate"	) == 0) { mopts->ca = true; sscanf(optarg, "%d", &mopts->c_sam); break;}
			if (strcmp(lopts[i].name, "reset"	) == 0) { mopts->re = true;					 break;}
			if (strcmp(lopts[i].name, "quiet"	) == 0) { mopts->quiet = true; break;}
			if (strcmp(lopts[i].name, "connect"	) == 0) { mopts->ne = true; 
				sscanf(strtok(optarg, ":"), "%s", mopts->n_hos);
				sscanf(strtok(NULL  , ","), "%s", mopts->n_por);}
				break;
		case '?' :
			fprintf(stderr, "unrecognized option \n");
			break;
		default:
			printf("?? getopt returned character code 0%o ??\n", c);
		}
	}
	if (optind < argc) {
		printf("non-option ARGV-elements: ");
		while (optind < argc)
			printf("%s ", argv[optind++]);
		printf("\n");
	}

	return 0;
}

void mpu_opt_pri(struct mpu_opt *mopts)
{
	printf("OPTIONS:\n");
	printf("CLKSEL:%5s %d\n", (mopts->dl ? "true" : "false"), mopts->dlpfv);
	printf("DLPF:%5s %d\n", (mopts->dl ? "true" : "false"), mopts->dlpfv);
	printf("SAMPLING_RATE:%5s %d\n", (mopts->sr ? "true" : "false"), mopts->smprt);
	printf("ACCEL_FULL_RANGE:%5s %d\n", (mopts->ar ? "true" : "false"), mopts->a_ran);
	printf("GYRO_FULL_RANGE:%5s %d\n", (mopts->gr ? "true" : "false"), mopts->g_ran);
	printf("DO_CALIBRATION:%5s %d\n", (mopts->ca ? "true" : "false"), mopts->c_sam);
	printf("DEVICE_RESET:%5s\n", 	  (mopts->re ? "true" : "false"));
	printf("DUMP_REGISTERS:%5s %s\n", (mopts->du ? "true" : "false"), mopts->d_fln);
	printf("NETWORK:%5s host:%s port:%s\n", (mopts->ne ? "true" : "false"), mopts->n_hos, mopts->n_por);
	printf("\n");
}

int mpu_opt_set(struct mpu_dev *dev, struct mpu_opt *mopts)
{
	if (mopts->re)
		mpu_ctl_reset(dev);

	if (mopts->cs)
		mpu_ctl_clocksource(dev, mopts->clksel);	

	if (mopts->dl)
		mpu_ctl_dlpf(dev, mopts->dlpfv);

	if (mopts->sr)
		mpu_ctl_samplerate(dev, mopts->smprt);

	if (mopts->ar)
		mpu_ctl_accel_range(dev, mopts->a_ran);

	if (mopts->gr)
		mpu_ctl_gyro_range(dev, mopts->g_ran);

	if (mopts->du)
		mpu_ctl_dump(dev,  mopts->d_fln);

	if (mopts->ca)
		mpu_ctl_calibration(dev);

	return 0;
}
