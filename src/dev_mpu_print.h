#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdio.h>
#include "dev_mpu.h"

void mpu_print_all(struct mpu_dev *dev, char *msg, char *buf);
void mpu_print_data  (struct mpu_dev *dev, int times);
void mpu_print_bias  (struct mpu_dev *dev);
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

