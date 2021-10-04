/*
 * empty buffer by dummy readingg
 */
void mpu_ctl_flush_fifo(int *fd)
{
	printf("%s\n", __func__);
	int16_t temp, count;
	while ( count > 0) {
		temp = mpu_comm_read_word(fd, FIFO_COUNT_H);
		count = temp >> 8;
		count |= (temp <<8);
		temp = mpu_comm_read_byte(fd, FIFO_R_W) <<8 ;
		temp |= mpu_comm_read_byte(fd,FIFO_R_W);
	}
}
