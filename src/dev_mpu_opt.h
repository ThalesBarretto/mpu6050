#ifdef __cplusplus
	extern "C" {
#endif

#ifndef _DEV_MPU_OPT_H_
#define _DEV_MPU_OPT_H_

#include "dev_mpu.h"
#include <getopt.h>

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
	unsigned int clksel;	/* clksel value			*/
	unsigned int dlpfv;	/* samplerate value		*/
	unsigned int smprt;	/* samplerate value		*/
	unsigned int a_ran;	/* accel range value		*/
	unsigned int g_ran;	/* gyro range value		*/
	unsigned int c_sam;	/* cal samples value		*/
	char d_fln[256];/* dump filename		*/
	char n_hos[256];/* network host			*/
	char n_por[16];	/* network port			*/
};


int mpu_opt_get(struct option *lopts, int argc, char **argv, struct mpu_opt *mopts);
int mpu_opt_set(struct mpu_dev *dev, struct mpu_opt *mopts);
void mpu_opt_pri(struct mpu_opt *mopts);

#endif /* _DEV_MPU_OPT_H_ */

#ifdef __cplusplus
	}
#endif
