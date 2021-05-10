/*
 * Write 1 Byte data to register
 */
void mpu_comm_write_byte(int *fd, uint8_t reg, uint8_t val)
{
	__s32 temp;
	temp = i2c_smbus_write_byte_data(*fd,reg,val);
//	printf("WRITE_BYTE %x %s - %x\n", reg, mpu_regnames[reg], val);
	if (temp != 0) {
		fprintf(stderr, "Error writing: %20s  %02x  code:%i\n", mpu_regnames[reg], val, temp);
		exit(1);
	}
}

