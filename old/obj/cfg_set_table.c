/*
 * Set and update config table
 */
void mpu_cfg_set_table(int *fd, uint8_t reg, uint8_t val)
{
	printf("%s\n", __func__);
	for (int i =1; i < mpu_cfg[0][0] ; i++)	{
		if ( mpu_cfg[i][0] == reg ) {
			mpu_cfg[i][1] = val;
		}
	}
	printf("%s calling get_table\n", __func__);
	mpu_cfg_get_table(fd);
}
