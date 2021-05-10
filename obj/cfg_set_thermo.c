/* Enable temperature reading*/
void mpu_cfg_set_thermo(int *fd)
{
//	printf("%s\n", __func__);
	uint8_t  fifov = (0x80 | mpu_cfg_read(fd,FIFO_ENABLE));
	mpu_cfg_set_table(fd, FIFO_ENABLE, fifov);
	mpu_cfg_get_table(fd);
}
