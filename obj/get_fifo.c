/*
 * Read data from buffer
 *
 * The only access to the FIFO buffer, keeps it in sync.
 *
 * After a sequece of data is read from the fifo buffer, the raw values are
 * scaled with respect to the range settings on each sensor and stored
 * into data[] which is used for further processing.
 *
 */
void mpu_get_fifo(int *fd)
{
//	printf("%s\n", __func__);
	uint16_t count;
	uint16_t many = 2*(raw[0]-1);
	uint16_t temp;
//	printf("Enabled: accel=%i gyro=%i temp=%i\n", a_en, g_en, t_en);
//	printf("Will read %i bytes - raw[0]=%i\n", many, raw[0]);
	do {
		temp = mpu_comm_read_word(fd, FIFO_COUNT_H);
/*
		count = temp >> 8;
		count |= (temp <<8);
*/
		count = (temp << 8) | (temp >> 8);
		if (count > 1000) {
			printf("BUFFER OVERFLOW -- empty buffer\n");
			mpu_ctl_flush_fifo(fd);
		}
	} while ( count < many  );
//	printf("read %i bytes\n", count);

	for (int i=1; i < raw[0]; i++){
//		printf("%i\n", i);
		raw[i] = mpu_comm_read_byte(fd, FIFO_R_W) <<8 ;
		raw[i] |= mpu_comm_read_byte(fd,FIFO_R_W);
	}

	// TODO: this is ugly, better use a rotation matrix.
	count = 1;
	/* fix accelerometer axis to match gyroscope */
	if ( a_en == 1 )	{
//		printf("Accel enabled - a_en = %i\n", a_en);
		raw[count] *= -1;
		count++;
		raw[count] *= -1;
		count++;
		raw[count] *= -1;
//		printf("Calculating |A|\n");
		}
	if ( t_en == 1)	{
//		printf("Thermo enabled - t_en = %i\n", t_en);
		count++;
	}
	if ( g_en == 1 ){
//		printf("Gyro enabled - g_en = %i\n", g_en);
		count++;
		count++;
		count++;
//		printf("Calculating |G|\n");
		G_magnitude = sqrt( (*Gx2) + (*Gy2) + (*Gz2) );
	}

	/* scale data */
	for (int i=1; i < raw[0]; i++){
		data[i][1] = data[i][0];
		data[i][0] = (raw[i]*scale[i]);
	}

	/* square data */
	if (a_en) {
		*Ax2 = *Ax * *Ax;
		*Ay2 = *Ay * *Ay;
		*Az2 = *Az * *Az;
		A_magnitude = sqrt( (*Ax2) + (*Ay2) + (*Az2) );
	}
	if (t_en){
		*Gx2 = *Gx * *Gx;
		*Gy2 = *Gy * *Gy;
		*Gz2 = *Gz * *Gz;
		G_magnitude = sqrt( (*Gx2) + (*Gy2) + (*Gz2) );

	}
//	printf("Incrementing sample\n");
	counter_samples++;
}
