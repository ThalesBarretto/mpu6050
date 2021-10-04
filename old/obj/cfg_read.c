/*
 * Read one config from table, return value
 */
 uint8_t mpu_cfg_read(int *fd, uint8_t reg)
{
	for (int i =1; i < mpu_cfg[0][0] ; i++)	{
		if ( mpu_cfg[i][0] == reg ) {
		 	return  mpu_cfg[i][1];
		}
	}
}
