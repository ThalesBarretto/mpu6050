/*
 * Write config table to registers
 * Use mpu_ctl_start() for proper handling.
 */
void mpu_cfg_write(int *fd)
{
	int temp;
	printf("\nWriting config registers\n");
	for (int i =1; i < mpu_cfg[0][0] ; i++)	{
		 mpu_comm_write_byte(fd, mpu_cfg[i][0], mpu_cfg[i][1]);
		/* wait some time */
		for (int i = 0; i < 50000; i++){};
	}
}
