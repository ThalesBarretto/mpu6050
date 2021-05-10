/*
 * Read 1 Byte data from register
 */
 uint8_t mpu_comm_read_byte(int *fd, uint8_t reg)
{
	uint8_t val;
	__s32 retvalue = i2c_smbus_read_byte_data(*fd, reg);
//	printf("READ_BYTE %x %s - %x\n", reg, mpu_regnames[reg], retvalue);
	if( retvalue < 0 ) {
		fprintf(stderr,"Error reading %20s %02x code: %i\n", mpu_regnames[reg], reg, retvalue );
		exit(1);
	} else 	return val = retvalue;
}

