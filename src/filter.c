// SPDX-License-Identifier: MIT
/* Copyright (C) 2021 Thales Antunes de Oliveira Barretto */
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <inttypes.h>
#include <tgmath.h>
#include <libmtx/mtx.h>
#include "filter.h"

int filter_init(struct mpu_dev *dev, struct mpu_flt_dat **flt)
{
	*flt = (struct mpu_flt_dat *)calloc(1, sizeof(struct mpu_flt_dat));

	(*flt)->dev = dev;

	(*flt)->anf = (struct mpu_ang *)calloc(1, sizeof(struct mpu_ang));
	(*flt)->ana = (struct mpu_ang *)calloc(1, sizeof(struct mpu_ang));
	(*flt)->ang = (struct mpu_ang *)calloc(1, sizeof(struct mpu_ang));
	(*flt)->anm = (struct mpu_ang *)calloc(1, sizeof(struct mpu_ang));

	angles_init(dev, &(*flt)->anf);
	angles_init(dev, &(*flt)->ana);
	angles_init(dev, &(*flt)->ang);
	angles_init(dev, &(*flt)->anm);

	(*flt)->isd = 1;		/* output angles in degrees	*/
	(*flt)->mtr = 0.5L;		/* accel motion treshold in g's	*/
	(*flt)->ant = 0.08L;		/* accel noise treshold in g's	*/

	(*flt)->gain = 0.8L;		/* gain parameter		*/
	(*flt)->gaa = 1 - (*flt)->gain; /* computed accelerometer gain	*/
	(*flt)->gag = (*flt)->gain;	/* computed gyroscope gain	*/
	return 0;
}

int filter_destroy(struct mpu_flt_dat *flt)
{
	angles_destroy(flt->anf);
	angles_destroy(flt->ana);
	angles_destroy(flt->ang);
	angles_destroy(flt->anm);
	free(flt);
	return 0;

}

int filter_update(struct mpu_flt_dat *flt)
{
	angles_accel(flt->ana);
	angles_gyro(flt->anf, flt->ang);
	if (fabs((*(flt->dev->AM) - 1) > flt->mtr)) {	/* motion detected */
		flt->anf->ean[0] =  flt->ang->ean[0];
		flt->anf->ean[1] =  flt->ang->ean[1];
	} else { /* fuse by complementary filter */
		flt->anf->ean[0] = (flt->gaa * flt->ana->ean[0]) + (flt->gag * flt->ang->ean[0]);
		flt->anf->ean[1] = (flt->gaa * flt->ana->ean[1]) + (flt->gag * flt->ang->ean[1]);
	}
	flt->anf->ean[2] =  flt->ang->ean[2];	/* psi angle only from gyro */
	flt->anf->rho = flt->ana->rho;
	return 0;
}
