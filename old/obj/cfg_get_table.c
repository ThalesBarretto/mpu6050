void mpu_cfg_get_table(int *fd)
{
	printf("%s\n", __func__);
	/* scan the config table for settings */
	for (int i =1; i < mpu_cfg[0][0]; i++) {
//		printf("%s %i searching %s\n", __func__, i, mpu_regnames[mpu_cfg[i][0]]);
		switch ( mpu_cfg[i][0] ) {
		case FIFO_ENABLE :
			/* CRUCIAL - raw[0] = active sensors + 1 */
			switch (mpu_cfg[i][1]) {
			case 0x08 : a_en = 1; g_en = 0; t_en = 0; raw[0] = 4; break;
			case 0x78 : a_en = 1; g_en = 1; t_en = 0; raw[0] = 7; break;
			case 0xF8 : a_en = 1; g_en = 1; t_en = 1; raw[0] = 8; break;
			case 0x88 : a_en = 1; g_en = 0; t_en = 1; raw[0] = 5; break;
			case 0x70 : a_en = 0; g_en = 1; t_en = 0; raw[0] = 4; break;
			case 0xF0 : a_en = 0; g_en = 1; t_en = 1; raw[0] = 5; break;
			case 0x80 : a_en = 0; g_en = 0; t_en = 1; raw[0] = 2; break;
			case 0x00 : a_en = 0; g_en = 0; t_en = 0; raw[0] = 1; break;
			default :
				printf("SHOULD NOT BE HERE found FIFO_ENABLE value: %x", mpu_cfg[i][1]);
				a_en = 0; g_en = 0; t_en = 0; raw[0] = 0;
				getchar();
				break;
			} break;
		case ACCEL_CONFIG :
			switch ( (mpu_cfg[i][1]) >> 3 ) {
			case 0x00 :
				accel_fullrange = 2.0;
				break;
			case 0x01 :
				accel_fullrange = 4.0;
				break;
			case 0x02 :
				accel_fullrange = 8.0;
				break;
			case 0x03 :
				accel_fullrange = 16.0;
				break;
			default :
				printf("SHOULD NOT BE HERE ACCEL_CONFIG value: %x", mpu_cfg[i][1]);
				getchar();
				break;
			} break;
		case GYRO_CONFIG :
			switch ( (mpu_cfg[i][1]) >> 3 ) {
			case 0x00 : gyro_fullrange = 250.0; break;
			case 0x01 : gyro_fullrange = 500.0; break;
			case 0x02 : gyro_fullrange = 1000.0; break;
			case 0x03 : gyro_fullrange = 2000.0; break;
			default :
				printf("SHOULD NOT BE HERE GYRO_CONFIGvalue: %x", mpu_cfg[i][1]);
				getchar();
				break;

			} break;
		case CONFIG :
			switch (mpu_cfg[i][1]) {
			case 0x00 :
				accel_bandwidth = 260.0;
				accel_delay = 0;
				gyro_bandwidth = 256.0;
				gyro_delay = 0.98;
				gyro_output_rate = 8000;
				break;
			case 0x01 :
				accel_bandwidth = 184.0;
				accel_delay = 2.0;
				gyro_bandwidth = 188.0;
				gyro_delay = 1.9;
				gyro_output_rate = 1000;
				break;
			case 0x02 :
				accel_bandwidth = 94.0;
				accel_delay = 3.0;
				gyro_bandwidth = 98;
				gyro_delay = 2.8;
				gyro_output_rate = 1000;
				break;
			case 0x03 :
				accel_bandwidth = 44.0;
				accel_delay = 4.9;
				gyro_bandwidth = 42.0;
				gyro_delay = 4.0;
				gyro_output_rate = 1000;
				break;
			case 0x04 :
				accel_bandwidth = 21.0;
				accel_delay = 8.5;
				gyro_bandwidth = 20.0;
				gyro_delay = 8.3;
				gyro_output_rate = 1000;
				break;
			case 0x05 :
				accel_bandwidth = 10.0;
				accel_delay = 13.8;
				gyro_bandwidth = 10.0;
				gyro_delay = 13.4;
				gyro_output_rate = 1000;
				break;
			case 0x06 :
				accel_bandwidth = 5.0;
				accel_delay = 19.0;
				gyro_bandwidth = 5.0;
				gyro_delay = 18.6;
				gyro_output_rate = 1000;
				break;
			case 0x07 :
				accel_bandwidth = 0;
				accel_delay = 0;
				gyro_bandwidth = 0;
				gyro_delay = 0;
				gyro_output_rate = 8000;
				break;
			default :
//				printf("SHOULD NOT BE HERE CONFIG value: %x", mpu_cfg[i][1]);
//				getchar();
				break;
			} break;
		default :
//			printf("Config not found\n");
//			getchar();
			break;
		}

	}

	/* THIS MUST BE DONE AFTER THE ABOVE STEPS
	 * Because the sampling frequency depends on TWO SETTINGS
	 * - the DLPF and the SMPLRT_DIV
	 *
	 * Thereforem sampling_rate and sampling_time are determined
	 * by a second order effect.
	 */
	for (int i =1; i < mpu_cfg[0][0] ; i++)	{
		if ( mpu_cfg[i][0] == SMPLRT_DIV ) {
			samplerate_divisor = (mpu_cfg[i][1] +1) ;
			sampling_rate = gyro_output_rate / samplerate_divisor ;
			sampling_time = 1 / sampling_rate;
		}
	}

	/*
	 * find LBS - Lowest Bit Lensitivity
	 *
	 * 16 bits signed data range
	 * max: 2⁽¹⁶⁻¹⁾   = +32768;
	 * min: 2⁽¹⁶⁻¹⁾-1 = -32767;
	 */
	accel_lbs = 32768.0/accel_fullrange;
	gyro_lbs = 32768.0/gyro_fullrange;

	mpu_cfg_get_scaling(fd);

	/* Associate data with meaningful names */
	int count = 1;
	if ( a_en == 1 ) {
		// 1
		Ax = &data[count][0];
		Ax2 = &squares[count];
		Ax_offset = &calibration_offsets[count];
		Ax_gain = &calibration_gains[count];
		Ax_driftrate = &calibration_driftrate[count];
		Ax_mean = &stats_mean[count];
		Ax_variance = &stats_variance[count];
		count++;
		// 2;
		Ay = &data[count][0];
		Ay2 = &squares[count];
		Ay_offset = &calibration_offsets[count];
		Ay_gain = &calibration_gains[count];
		Ay_driftrate = &calibration_driftrate[count];
		Ay_mean = &stats_mean[count];
		Ay_variance = &stats_variance[count];
		count++;
		// 3
		Az = &data[count][0];
		Az2 = &squares[count];
		Az_offset = &calibration_offsets[count];
		Az_gain =&calibration_gains[count];
		Az_driftrate = &calibration_driftrate[count];
		Az_mean = &stats_mean[count];
		Az_variance = &stats_variance[count];
		count++;
	}
	if ( t_en == 1 )	{
		Thermo = &data[count][0];
		Thermo_offset = &calibration_offsets[count];
		Thermo_gain =&calibration_gains[count];
		Thermo_driftrate = &calibration_driftrate[count];
		Thermo_mean = &stats_mean[count];
		Thermo_variance = &stats_variance[count];
		count++;
	}
	if ( g_en == 1 )	{
		Gx = &data[count][0];
		Gx2 = &squares[count];
		Gx_offset = &calibration_offsets[count];
		Gx_gain =&calibration_gains[count];
		Gx_driftrate = &calibration_driftrate[count];
		Gx_mean = &stats_mean[count];
		Gx_variance = &stats_variance[count];
		count++;
		Gy = &data[count][0];
		Gy2 = &squares[count];
		Gy_offset = &calibration_offsets[count];
		Gy_gain =&calibration_gains[count];
		Gy_driftrate = &calibration_driftrate[count];
		Gy_mean = &stats_mean[count];
		Gy_variance = &stats_variance[count];
		count++;
		Gz = &data[count][0];
		Gz2 = &squares[count];
		Gz_offset = &calibration_offsets[count];
		Gz_gain =&calibration_gains[count];
		Gz_driftrate = &calibration_driftrate[count];
		Gz_mean = &stats_mean[count];
		Gz_variance = &stats_variance[count];
	}

	// TODO: Why?????
	for (int i=1; i < raw[0]; i++){
		data[i][1] = data[i][0];
		data[i][0] = (raw[i]*scale[i]);
	}
}
