/*
 * Set SLEEP bit, wich puts device in sleep low power mode
 * //TODO: write to configuration table
 */
void mpu_ctl_sleep(int *fd)
{
	mpu_comm_write_byte(fd, PWR_MGMT_1, 0x40);
}
