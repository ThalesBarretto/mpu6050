/*
 * Read 2 Byte data (1 Word) from register
 */

 uint16_t mpu_comm_read_word(int *fd, uint8_t reg)
{
	uint8_t val;
	__s32 retvalue = i2c_smbus_read_word_data(*fd, reg);
//	printf("READ_WORD %x %s - %x\n", reg, mpu_regnames[reg], retvalue);
	if( retvalue < 0 ) {
		printf( "Error writing data - code: %i\n", retvalue );
		exit( 1 );
	} else 	return val = retvalue;
}
