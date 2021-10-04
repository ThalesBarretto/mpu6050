/*
 * Checks config table against the actual device registers
 */
void mpu_cfg_validate(int *fd)
{
	int failed = 0;
	uint8_t temp[16][3];
	temp[0][0] = mpu_cfg[0][0];
	for (int i = 1; i < mpu_cfg[0][0]; i++)	{
		temp[i][0] = mpu_cfg[i][0];
		temp[i][1] = mpu_cfg[i][1];
		temp[i][2] = mpu_comm_read_byte(fd, mpu_cfg[i][0]);
	}
	/* 4 columns
	 * 20 1 = Register name
	 * 6  2 = Expec.
	 * 6  3 = Actual
	 * 5  4 = PASS/FAIL
	 */
	printf( "=================================================\n"  \
		"       Config Validation\n" \
		"-------------------------------------------------\n");

	for (int i = 1; i < temp[0][0] ; i++) {
		printf(	"%20s %02x vs %02x  ",\
			mpu_regnames[temp[i][0]], \
			temp[i][1], \
			temp[i][2] );

		if ( failed = (temp[i][1] != temp[i][2]) )
			printf( "%-5s\n", "FAIL");
		else
			printf( "%-5s\n", "PASS");
	}
	if (failed) {
		printf( "\n   CONFIG VALIDATION FAIL \n" );
		getchar();
	} else {
		printf( "\n   CONFIG VALIDATION PASS \n" );
	}
	printf(	"-------------------------------------------------\n" );

}
