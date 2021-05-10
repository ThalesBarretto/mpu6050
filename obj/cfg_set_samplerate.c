/*
 * Use to set the desired sampling rate.
 *
 * NOTE: too high will cause overflow
 *  Recommended: sampling_rate < 250
 */
void mpu_cfg_set_samplerate(int *fd, double sampling_rate)
{
	printf("%s\n", __func__);
	mpu_cfg_get_table(fd);
	uint8_t smplrt_div = (gyro_output_rate / sampling_rate) -1;
	mpu_cfg_set_table(fd, SMPLRT_DIV, smplrt_div);
}
