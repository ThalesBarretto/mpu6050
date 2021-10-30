#include <stdio.h>
#include <string.h>
#include <libmpu6050/mpu6050_core.h>
#include "demo_opt.h"

int mpu_opt_get(struct option *opts, int argc, char **argv, struct mpu_opt *mopts)
{
	int c;
	int i = 0;
	while ((c = getopt_long(argc, argv, "", opts, &i)) != -1) {
		switch(c) {
		case 0:
			if (strcmp(opts[i].name, "clksel"	) == 0) { mopts->cs = true; sscanf(optarg, "%u", &mopts->clksel); break;}
			if (strcmp(opts[i].name, "dlpf"		) == 0) { mopts->dl = true; sscanf(optarg, "%u", &mopts->dlpfv ); break;}
			if (strcmp(opts[i].name, "rate"		) == 0) { mopts->sr = true; sscanf(optarg, "%u", &mopts->smprt ); break;}
			if (strcmp(opts[i].name, "arange"	) == 0) { mopts->ar = true; sscanf(optarg, "%u", &mopts->a_ran ); break;}
			if (strcmp(opts[i].name, "grange"	) == 0) { mopts->gr = true; sscanf(optarg, "%u", &mopts->g_ran ); break;}
			if (strcmp(opts[i].name, "dump"		) == 0) { mopts->du = true; sscanf(optarg, "%s",  mopts->d_fln ); break;}
			if (strcmp(opts[i].name, "calibrate"	) == 0) { mopts->ca = true; break;}
			if (strcmp(opts[i].name, "reset"	) == 0) { mopts->re = true; break;}
			if (strcmp(opts[i].name, "quiet"	) == 0) { mopts->quiet = true; break;}
			if (strcmp(opts[i].name, "connect"	) == 0) { mopts->ne = true; 
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

