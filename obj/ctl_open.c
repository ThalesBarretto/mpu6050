/*
 * Open a file associated with the bus driver, attach the slave device.
 * Test for communication.
 *
 */
void mpu_ctl_open(int *fd, int adapter_nr)
{
	char filename[20];
	snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
	*fd = open("/dev/i2c-1", O_RDWR);
	ioctl(*fd, I2C_SLAVE, MPU6050_ADDR);
	signal(SIGINT,mpu_ctl_siginterrupt_handler);
	mpu_ctl_ping(fd);
	mpu_ctl_wake(fd);
	mpu_ctl_setclk(fd,0);
}
