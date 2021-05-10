
/*
 * Print config table data to the screen
 */
void mpu_print_config(int *fd)
{
	printf(" CONFIG	\tVALUE\n");
	printf(" accel_fullrange \t%-8.8f\n", accel_fullrange);
	printf(" accel_lbs \t%-8.8f\n", accel_lbs);
	printf(" accel_bandwidth \t%-8.8f\n", accel_bandwidth);
	printf(" accel_delay \t%-8.8f\n", accel_delay);
	printf(" gyro_fullrange \t%-8.8f\n", gyro_fullrange);
	printf(" gyro_lbs \t%-8.8f\n", gyro_lbs);
	printf(" gyro_bandwidth \t%-8.8f\n", gyro_bandwidth);
	printf(" gyro_output_rate \t%-17u\n" ,gyro_output_rate);
	printf(" sampling_rate \t%-8.8f\n", sampling_rate);
	printf(" samplerate_divisor \t%-17u\n", samplerate_divisor);
}
