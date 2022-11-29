// SPDX-License-Identifier: MIT
/* Copyright (C) 2021 Thales Antunes de Oliveira Barretto */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <libmpu6050/mpu6050_core.h>
#include <libmpu6050/mpu6050_regs.h>
#include "angles.h"

void snprint_data(struct mpu_dev *dev, char *msg, char *buf)
{
	sprintf(buf, "%5llu ", dev->samples);
	strcat(msg, buf);
	//sprintf(buf, "Hz:%3.0lf ", dev->sr);
	//strcat(msg,buf);
	//sprintf(buf,  "T:%+.1lf",*(dev->t));
	//strcat(msg,buf);
	sprintf(buf, " |A|=%lf ", *(dev->AM) );
	strcat(msg,buf);
	sprintf(buf, "Ax:%+lf Ay:%+lf Az:%+lf ", *(dev->Ax), *(dev->Ay), *(dev->Az));
	strcat(msg,buf);
	sprintf(buf, "Gx:%+lf ", *(dev->Gx));
	strcat(msg,buf);
	sprintf(buf, "Gy:%+lf ", *(dev->Gy));
	strcat(msg,buf);
	sprintf(buf, "Gz:%+lf ", *(dev->Gz));
	strcat(msg,buf);
}

void snprint_angle(struct mpu_ang *ang, char *msg, char *buf)
{
	sprintf(buf, "phi:%+.2Lf ", ang->ean[0]);
	strcat(msg, buf);

	sprintf(buf, "the:%+.2Lf ", ang->ean[1]);
	strcat(msg, buf);

	sprintf(buf, "psi:%+.2Lf ", ang->ean[2]);
	strcat(msg, buf);
}
