/*
 * Get calibration data.
 */
void mpu_get_calibration(int *fd, int seconds)
{
	int times = seconds * sampling_rate;
	printf("Calibrating in %i seconds (%i samples) - Fs:%f Ts%f\n", seconds, times, sampling_rate, sampling_time);
	/* A HUGE amount of data */
	double caldata[16][times];
	double caldata_offsets[16];

	/* flush buffer */
	mpu_ctl_flush_fifo(fd);
	/* wait 200ms for stabilization */
	for( double t = 0.200 * sampling_rate; t > 0; t--) {
		mpu_get_fifo(fd);
	}

	printf("Gathering data...");
	/* read */
	for (int k = 0; k < times; k++) {
		caldata[0][k] = k*sampling_time;
		mpu_get_fifo(fd);
		for (int i = 1; i < raw[0]; i++) {
			caldata[i][k] = data[i][0];
			calibration_offsets[i] += data[i][0];
			calibration_driftrate[i] += data[i][0] - data[i][1];
			squares[i] = data[i][0]*data[i][0];
		}
		if (a_en == 1) {
			// printf("Accumulate G\n");
			calibration_gravity += sqrt( (*Ax2) + (*Ay2) + (*Az2) );
		}
	}
	printf("done\n");

	printf("Calibrating Linear least square fitting...");
	for (int i = 1; i < raw[0]; i++) {
		gsl_fit_linear(caldata[0], 1, \
			caldata[i], 1, \
			times,
			&mpu_gsl_fit_linear_c0[i], \
			&mpu_gsl_fit_linear_c1[i], \
			&mpu_gsl_fit_linear_cov00[i], \
			&mpu_gsl_fit_linear_cov01[i], \
			&mpu_gsl_fit_linear_cov11[i], \
			&mpu_gsl_fit_linear_sumsq[i]);
	}
	printf("done\n");


	/* normalize */
	for (int i = 1; i < raw[0]; i++) {
		calibration_offsets[i] /= times;
		calibration_driftrate[i] /= times;
		stats_mean[i] = calibration_offsets[i];
	}
	calibration_gravity /= times;
	printf("Galibration Gravity = %f \n", calibration_gravity);


	/* setup */
	/* Accelerometers won't bias, only scale */
	if ( a_en == 1 ) {
	*Ax_gain = 1/calibration_gravity;
	*Ay_gain = 1/calibration_gravity;
	*Az_gain = 1/calibration_gravity;
	*Ax_offset = 0;
	*Ay_offset = 0;
	*Az_offset = 0;
	*Ax_driftrate = 0;
	*Ay_driftrate = 0;
	*Az_driftrate = 0;
	}
	/* Thermo has fixed calibration */
	if ( t_en == 1) {
	*Thermo_gain = 1;
	*Thermo_offset = -36.5;
	*Thermo_driftrate = 0;
	}
	if ( g_en == 1 ) {
	/* Gyroscopes won't scale */
	*Gx_gain = 1;
	*Gy_gain = 1;
	*Gz_gain = 1;
	}


	printf("Getting calibration ypr...\n");
	for (int k= 0; k < times; k++) {
		mpu_get_fifo(fd);
		mpu_get_angles(fd);
		for (int i =1; i < 4; i++) {
			calibration_ypr_accel[i] += ypr_accel[i];
		}
	}
	for (int i = 1; i < 4; i++) {
		calibration_ypr_accel[i] /= times;
		ypr_gyro[i] = calibration_ypr_accel[i];
	}
	printf("done\n");

	printf( "Getting calibration statistics..." );
	/* Accumulate the square of the deviation */
	for (int k = 0; k < times; k++) {
		for (int i = 1; i < raw[0]; i++) {
			stats_variance[i] += (caldata[i][k] - stats_mean[i])*(caldata[i][k] - stats_mean[i]);
		}
	}
	/* divide by (n-1), take the square root */
	for (int i = 1; i < raw[0]; i++) {
		stats_variance[i] /= (double)(times -1);
		stats_variance[i] = sqrt(stats_variance[i]);
	}
	printf( "done\n" );
}
