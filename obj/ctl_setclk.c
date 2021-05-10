/*
 * Configure the clock source setting [0-7]
 * //TODO: write to configuration table
 */
void mpu_ctl_setclk(int *fd, uint8_t clksrc)
{
	clksrc &= 0x07;
	clksrc |= mpu_comm_read_byte(fd, PWR_MGMT_1);
	mpu_comm_write_byte(fd, PWR_MGMT_1, clksrc);
}
