/*
 * Clear SLEEP bit, waking up the device
 * //TODO: write to configuration table
 */
void mpu_ctl_wake(int *fd)
{
	uint8_t value = (~0x40) & mpu_comm_read_byte(fd, PWR_MGMT_1);
	/* recover from sleep mode */
	mpu_comm_write_byte(fd, PWR_MGMT_1, value);
	/* recover sensors from standby */
	mpu_comm_write_byte(fd, PWR_MGMT_2, 0x00);

}
