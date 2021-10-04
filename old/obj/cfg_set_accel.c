/*
 * Configure the gryoscope range and enables FIFO buffering for its readings
 *
 * Valid ranges:
 * +-2  g use 2
 * +-4  g use 4
 * +-8  g use 8
 * +-16 g use 16
 */
void mpu_cfg_set_accel(int *fd, uint16_t range)
{
//	printf("%s\n", __func__);
	uint8_t  fifov = (0x08 | mpu_cfg_read(fd,FIFO_ENABLE));
	uint8_t regv;
//	printf("%s range=%i regv=%x\n", __func__, range, regv);
	switch (range) {
	case 2 :	regv = 0x00; break;
	case 4 : 	regv = 0x01; break;
	case 8 :	regv = 0x02; break;
	case 16 :	regv = 0x03; break;
	default : 	regv = 0X00;
			fifov &= ~0x08;
			break;
	}
//	printf("%s got  regv=%x\n", __func__, regv);
//	printf("%s calling set_table FIFO_ENABLE\n", __func__);
	mpu_cfg_set_table(fd, FIFO_ENABLE, fifov);

//	printf("%s calling set_table ACCEL_CONFIG - passing regv=%x\n", __func__, regv);
	mpu_cfg_set_table(fd, ACCEL_CONFIG, regv << 3);

	printf("%s calling get_table\n", __func__);
	mpu_cfg_get_table(fd);
}
