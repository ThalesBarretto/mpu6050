/*
 * Reads "n" Bytes from mpu
 * Read the contents of registers starting at "reg"
 * on the target device with "addr", and puts as many
 * bytes into "val" as the size of "val".
 *
 * Parameters
 *
 * file - file descriptor associated to an i2c bus master
 * 	which is returned by "open("/dev/i2c-1", O_RDWR)"
 *
 * addr - the address of the target slave device in the bus
 *
 * reg	- the address of the target register on target device
 *
 * val - receives the contents of the target register
 * 	 val[0] contains the number of bytes to be read
 *
 */

void mpu_comm_read_burst(int *fd)
{
	printf("Entering %s\n", __func__);

	/* Prepare to read Accel  */
	uint8_t val[7];
	mpu_cfg_validate(fd);
	printf("Calling mpu_rnB...\n");
	mpu_rnB(fd,ACCEL_XOUT_H, val, 0x06);

	printf("Press a key to exit...\n");
	getchar();
	close(*fd);
	exit(0);
}
int mpu_rnB(int *fd,
//	unsigned char addr,  USE MPU6050_ADDR
	uint8_t reg,
	uint8_t *val,
	uint8_t n)
{
	printf("Entering %s - arguments: fd:%i reg:%02x val:%p, n%02x \n", __func__, fd, reg, val, n);
	printf("creating packets\n");
	/*
	 * We need to pass this as argument to the ioctl.
	 *
	 * Now, however, we must have 2 messages, one to
	 * send the address of the target register, other to
	 * store the contents of the target register.
	 *
	 * SEE THE I2C SPECIFICATION - MAYBE INCORRECT
	 * This is due to structure of the i2c messaging.
	 * In order to read a register, we do:
	 * 1-START->WRITE:ADDR+WRITE->wait for ACK
	 * 2->WRITE->REGADDR->wait for ACK
	 * 3->STOP
	 * 4->START->WRITE:ADDR+READ->wait for ACK
	 * 5->READ:value->send ACK
	 * 6->READ:value->send ACK ...
	 * 	7-->if done,send NACK
	 *	8->STOP
	 */
	struct i2c_rdwr_ioctl_data packets;

	/*
	 * In order to read a register, we first do a "dummy write" by writing
	 * 0 bytes to the register we want to read from.  This is similar to
	 * the packet in set_i2c_register, except it's 1 byte rather than 2.
	 *
	 * The first message contains the "write" part
	 * The second message contains the "read" part
	 */
	struct i2c_msg messages[2];
	uint8_t outbuf	  = reg;
	messages[0].addr  = MPU6050_ADDR;
	messages[0].flags = 0;
	messages[0].len   = sizeof(outbuf);
	messages[0].buf   = &outbuf;
	uint8_t inbuf[n];
	messages[1].addr  = MPU6050_ADDR;
	messages[1].flags = I2C_M_RD /* | I2C_M_NOSTART */ ;
	messages[1].len   = sizeof(inbuf);
	messages[1].buf   = inbuf;

	/*
	 * Send the request to the kernel and get the result back
	 */
	printf("building packet\n");
	packets.msgs      = messages;
	packets.nmsgs     = 2;

	printf("sending ioctrl\n");

	/*
	 * ioctl(file, I2C_RDWR, struct i2c_rdwr_ioctl_data *msgset)
	 *	Do combined read/write transaction without stop in between.
	 *	Only valid if the adapter has I2C_FUNC_I2C.
	 *
	 * The argument is a pointer to a:
	 *
	 *   struct i2c_rdwr_ioctl_data {
	 *	struct i2c_msg *msgs;		// ptr to array of simple messages
      	 *	int nmsgs;			//number of messages to exchange
    	 *	}
	 *
	 */

	if(ioctl(*fd, I2C_RDWR, &packets) < 0) {
		perror("Unable to send data");
		getchar();
		return 1;
	}

	/*
	 * If everything went ok, copy the reading to "val"
	 */
	printf("This is the read data:\n");
	/* *val = inbuf; */
	for (int i=0; i<n; i++){
	printf("inbuf[i]=%#02x\n",inbuf[i]);
	}
	getchar();
	return 0;
}
