/*
 * Configure the gryoscope range and enables FIFO buffering for its readings
 *
 * Valid ardumunts:
 * +-250 deg/s use 250
 * +-500 deg/s use 500
 * +-1000 deg/s use 1000
 * +-2000 deg/s use 2000
 */
void mpu_cfg_set_gyro(int *fd, uint16_t range)
{
	printf("%s\n", __func__);
	uint8_t fifov = (0x70 | mpu_cfg_read(fd, FIFO_ENABLE));
	uint8_t regv;
	switch (range)	{
	case 250 :	regv = 0x00; break;
	case 500 : 	regv = 0x01; break;
	case 1000 :	regv = 0x02; break;
	case 2000 :	regv = 0x03; break;
	default : 	regv = 0x00;
			fifov &= ~0x70;	break;
	}
	mpu_cfg_set_table(fd, FIFO_ENABLE, fifov);
	mpu_cfg_set_table(fd, GYRO_CONFIG, regv << 3);
	mpu_cfg_get_table(fd);
}

/*
 * Configure the gryoscope range and enables FIFO buffering for its readings
 *
 * Valid ranges:
 * +-2  g use 2
 * +-4  g use 4
 * +-8  g use 8
