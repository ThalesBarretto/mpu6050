/*
 * Handles the setup of "scale" array
 *
 * Each entry is detemined by the range
 * settings stored in the config table
 */
void mpu_cfg_get_scaling(int *fd)
{
	int count = 1;
	if( a_en == 1)
	{
		for (int i=0; i < 3; i++) {
			scale[count] =1/accel_lbs;
			count++;
		}
	}

	if( t_en == 1)
	{
		scale[count]=1/340;
		count++;
	}
	if( g_en == 1)
	{
		for (int i=0; i < 3; i++) {
		 	scale[count] =1/gyro_lbs;
			count++;
		}
	}
}
