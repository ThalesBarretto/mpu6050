/*
 * Build statistics from scaled data.
 */
void mpu_get_statistics(int *fd)
{
	double prev[8];

	for (int i=1; i < raw[0]; i++) {
		prev[i]  = delta[i];

		/* read last data*/
		stats_mean[i] = (data[i][0] + stats_mean[i])/2;
		delta[i] = data[i][0] - stats_mean[i];
		stats_variance[i]= (prev[i]*prev[i])+(delta[i]*delta[i]);
	}
}
