void mpu_ctl_start(int *fd)
{
	mpu_ctl_reset(fd);
	printf("%s\n", __func__);
	mpu_cfg_get_table(fd);
	mpu_cfg_write(fd);
	mpu_ctl_ping(fd);
	mpu_cfg_validate(fd);
	mpu_ctl_flush_fifo(fd);
}
