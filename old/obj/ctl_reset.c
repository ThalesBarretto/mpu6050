/*
 * Perform a device reset and wait for it to complete.
 */
void mpu_ctl_reset(int *fd)
{
	uint8_t value = 0x80 | mpu_comm_read_byte(fd, PWR_MGMT_1);
	mpu_comm_write_byte(fd, PWR_MGMT_1, value);
	for (int i = 0; i < 50000; i++){};
}
