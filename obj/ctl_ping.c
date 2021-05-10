/*
 * Verify that the device is responding
 */
void mpu_ctl_ping(int *fd)
{
	uint8_t whoami = mpu_comm_read_byte(fd,  WHO_AM_I);

	if ( whoami != MPU6050_ADDR )
	{
		printf("ERROR: WHO_AM_I | response: %02x expected:%02x\n", whoami, MPU6050_ADDR);
		getchar();
		exit(1);
	}
}
