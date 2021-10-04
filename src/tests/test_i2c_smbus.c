#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <limits.h>
#include <inttypes.h>

#include <stdio.h>		/* for perror() */
#include <unistd.h>		/* for write() */
#include <errno.h>		/* for errno */

#include <sys/ioctl.h>		/* for ioctl() */
#include <sys/stat.h>		/* for open() */
#include <sys/types.h>		/* for open() */
#include <fcntl.h>		/* for open() */

#include <linux/i2c-dev.h>	/* for i2c_smbus */
#include <i2c/smbus.h>		/* for i2c_smbus */
#include "mpu6050.h"		/* for register definitions */
int main(int argc, char *argv[])
{
	int file;
	int adapter_nr = 1;	/* probably dynamically determined */
	char filename[20];

	snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);

	if ((file = open (filename, O_RDWR)) < 0) {
		perror("Couldn't open device");
		exit(EXIT_FAILURE);
	} else {
		printf("open(): opened %s with fd %d\n", filename, file);
	}

	int addr = 0x68;	/* i2c bus addres */
	if ((ioctl(file, I2C_SLAVE, addr) < 0)) {
		perror("Couldn't set device address");
		exit(EXIT_FAILURE);
	} else {
		printf("ioctl(): Set slave address %#02x\n", addr);
	}

	__u8 reg;	/* device register to access */
	__u8 val;	/* value to be written to */
	__s32 res;
	char buf[10];

	/* Using SMBUS commands */
	reg = WHO_AM_I;
	if ((res = i2c_smbus_read_byte_data(file, reg)) < 0) {
		perror("Couldn't read reg");
		exit(EXIT_FAILURE);
	} else {
		printf("i2x_smbus_read_byte_data() :");
		printf("read %0#4x from register %0#2x\n", res, reg);
	}
	reg = PWR_MGMT_1;
	if ((res = i2c_smbus_read_byte_data(file, reg)) < 0) {
		perror("Couldn't read reg");
		exit(EXIT_FAILURE);
	} else {
		printf("i2x_smbus_read_byte_data() :");
		printf("read %0#4x from register %0#2x\n", res, reg);
	}
	val = 0x03;
	if ((res = i2c_smbus_write_byte_data(file, reg, val)) < 0) {
		perror("Couldn't write reg");
		exit(EXIT_FAILURE);
	} else {
		printf("i2x_smbus_write_byte_data() :");
		printf("written %0#4x to register %0#2x\n", val, reg);
	}
	sleep(1);
	if ((res = i2c_smbus_read_byte_data(file, reg)) < 0) {
		perror("Couldn't read reg");
		exit(EXIT_FAILURE);
	} else {
		printf("i2x_smbus_read_byte_data() :");
		printf("read %0#4x from register %0#2x\n", res, reg);
	}
}

/* 
 * READS
 * __s32 i2c_smbus_read_byte_data(	int file, __u8 command)
 * __s32 i2c_smbus_read_word_data(	int file, __u8 command)
 * __s32 i2c_smbus_read_block_data(int file, __u8 command, __u8 *values)
 *
 * parameters:	command: register to read from
 *		values: destination buffer
 * success: ({byte,word}_data): data received
 * 	     block_data: number bytes received
 * failure: -1, sets errno; 
 *
 *
 * WRITES
 * __s32 i2c_smbus_write_byte_data(int file,__u8 command,__u8 value)
 * __s32 i2c_smbus_write_word_data(int file,__u8 command,__u16 value)
 * __s32 i2c_smbus_write_block_data(int file,__u8 command,__u8 length,__u8 *values)
 * 
 * parameters 	file/client: handle to slave device
 *		command: register to write to
 *		value{s}: value{s} to be written
 * success: 0 (zero)
 * failure: negative errno; 
 *
 * IOCTLS
 * ioctl(file, I2C_FUNCS, unsigned long *funcs)
 * struct i2c_rdwr_ioctl_data {
 * 	struct i2c_msg *msgs;
 * 	int nmsgs;
 * }
 * ioctl(file, I2C_RDWR, struct i2c_rdwr_ioctl_data *msgset)
 */
