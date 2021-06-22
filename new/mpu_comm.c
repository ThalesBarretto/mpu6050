int mpu_comm_read_byte(	const int restrict *fd,
       			const uint8_t reg, 
			uint8_t restrict *val)
{
	if ((fd == NULL) || (val == NULL)) /* null pointer */
		return -1;

	__s32 retvalue = i2c_smbus_read_byte_data(*fd, reg);
	if( retvalue < 0 ) /* read errror, check errno */
		return -1;

	*val = (uint8_t)retval;
	return 0;
}

int mpu_comm_write_byte(const int restrict *fd,
       			const uint8_t reg, 
			const uint8_t val)
{
	if (fd == NULL) /* null pointer */
		return -1;

	if((i2c_smbus_write_byte_data(*fd, reg, val)) < 0)
	       return -1; /* write error, check errno */

	return 0;
}

int mpu_comm_read_word(	const int restrict *fd,
       			const uint8_t reg, 
			uint16_t restrict *val)
{
	if ((fd == NULL) || (val == NULL)) /* null pointer */
		return -1;

	__s32 retvalue = i2c_smbus_read_word_data(*fd, reg);
	if( retvalue < 0 ) /* read errror, check errno */
		return -1;

	*val = (uint16_t)retval;
	return 0;
}
int mpu_comm_read_bytes(const int restrict *fd,
       			const uint8_t reg, 
			uint8_t restrict *buffer
			const size_t bytes)
{
	if ((fd == NULL) || (buffer == NULL)) /* null pointer */
		return -1;
	if ((bytes <= 0) || (bytes > 32)) /* out ou bounds */
		return -1;


	__s32 retvalue = i2c_smbus_read_i2c_block_data(*fd, reg, buffer);
	if (retvalue < 0 ) /* read errror, check errno */
		return -1;
	if (retvalue < bytes ) /* read less than requested */
		return -1;
	if (retvalue > bytes ) /* fatal: out of bounds write */
		exit(EXIT_FAILURE);	

	return 0;
}

