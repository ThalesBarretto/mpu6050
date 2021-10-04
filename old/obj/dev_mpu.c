#include <string.h>	/* for memcpy */

#ifndef MPU6050_ADDR
#define MPU6050_ADDR 0x68
#endif

struct mpu_dev {
	/* internal data - managed through special handlers */
	int		*bus;		/* bus file decriptor			*/
	uint8_t		addr;		/* device i2c bus address		*/
	uint16_t	fifomax;	/* fifo buffer capacity in bytes	*/
	uint16_t	fifocnt;	/* bytes available in fifo		*/
	struct mpu_cfg	*cfg;		/* config register state		*/
	struct mpu_dat	*dat;		/* sensor readings			*/
	struct mpu_cal	*cal;		/* calibration data			*/
	/* readable config - result of special handlers */
	uint8_t		aen, gen, ten;	/* sensor enabled state			*/
	uint16_t	gor;		/* gyro output rate			*/
	double	sr, st, sdiv;		/* sampling rate, time, rate divisor	*/
	double	afr, albs, abdw, adly;	/* range, lbs, bandwidth, delay		*/
	double	gfr, glbs, gbdw, gdly;
	unsigned long long samples;	/* sample counter			*/
	/* readable data */
	double	*Ax, *Ax2, *Axo, *Axg, *Axm, *Axv, *Axd;
	double	*Ay, *Ay2, *Ayo, *Ayg, *Aym, *Ayv, *Ayd;
	double	*Az, *Az2, *Azo, *Azg, *Azm, *Azv, *Azd;
	double	*Gx, *Gx2, *Gxo, *Gxg, *Gxm, *Gxv, *Gxd;
	double	*Gy, *Gy2, *Gyo, *Gyg, *Gym, *Gyv, *Gyd;
	double	*Gz, *Gz2, *Gzo, *Gzg, *Gzm, *Gzv, *Gzd;
	double	*t, 	    *to,  *tg,  *tm,  *tv,  *td;
}

struct mpu_cal {
	//TODO make specific initializer: gravity = 1; gains =1, offsets =0;
	double	gra;			/* mean(sqrt(ax2,ay2,az2)[])		*/
	double	off[16];		/* ax - mean(ax[n]), ...		*/
	double	gai[16];		/* ax[expec_1g]/ax[measured_1g], ...	*/
	double	dri[16];		/* delta(mean(ax[n])/delta(time), ...	*/
};

struct mpu_dat {
	int16_t	raw[16];	/* raw sensor data	*/
	double	scl[16];	/* scaling factors	*/
	double	dat[16][2];	/* scaled data		*/
	double	squ[16];	/* squared data		*/
	double	mea[16];	/* data mean		*/
	double	var[16];	/* data variance	*/
};

struct mpu_cfg {
	uint8_t	cfg[11][2];	/* configuration state	*/
};

const struct mpu_cfg mpu6050_defcfg {
       	{	{ sizeof(mpu6050_defcfg.cfg / sizeof(mpu6050_defcfg.cfg[0],
		  sizeof(mpu6050_defcfg.cfg[0]}, /* lines, columns */
		{ PWR_MGMT_1,   0x03},	/* power on, temp enabled, clock gyro_z */
		{ PWR_MGMT_2,   0x00},	/* no standby, full on			*/
		{ CONFIG,       0x00},	/* dlpf off				*/
		{ SMPLRT_DIV,   0x00},	/* samplarete divisor = 1 		*/
		{ ACCEL_CONFIG, 0x00},	/* +-2g 				*/
		{ GYRO_CONFIG,  0x00},	/* +-250 deg/s 				*/
		{ INT_PIN_CFG,  0x00},	/* interrupts disabled			*/
		{ INT_ENABLE,   0x00},	/* interrupts disabled			*/
		{ USER_CTRL,    0x60},	/* fifo enabled, aux i2c master mode	*/
		{ FIFO_ENABLE,  0x00}	/* nothing bufferd by std		*/
	}
};


int mpu_comm_read_byte(	const int restrict *fd,
       			const uint8_t reg, 
			uint8_t restrict *val)
{
	if ((fd == NULL) || (val == NULL)) /* null pointer */
		return -1;

	__s32 retvalue = i2c_smbus_read_byte_data(*fd, reg);
	if( retvalue < 0 ) /* read errror */
		return -1;

	*val = (uint8_t)retval;
	return 0;
}

int mpu_ctl_wake(struct mpu_dev *dev)
{
	if ((dev == NULL) || (dev->bus == NULL)) 
		return -1;

	uint8_t	val;
	if ((mpu_comm_read_byte(dev->bus, PWR_MGMT_1, &val)) < 0)
		return -1;

	val &= ((uint8_t)~0xE0); /* clear DEVICE_RESET, SLEEP, CLYCLE bits */
	if ((mpu_comm_write_byte(dev->bus, PWR_MGMT_1, val)) < 0)
		return -1;
	/* disables any standby */
	if ((mpu_comm_write_byte(dev->bus, PWR_MGMT_2, 0x00)) < 0)
		return -1;


	return 0;
}

int mpu_ctl_set_CLKSEL(struct mpu_dev *dev, uint8_t clksrc)
{
	if ((dev == NULL) || (dev->bus == NULL)) 
		return -1;
	
	uint8_t	val;
	if ((mpu_comm_read_byte(dev->bus, PWR_MGMT_1, val)) < 0)
		return -1;


	val &= (~0x07);	/* mask bits */
	val |= clksrk;	/* set  bits */
	if ((mpu_comm_write_byte(dev->bus, PWR_MGMT_1, val)) < 0)
		return -1;

	/* wait at least 100ms */
	struct timespec halfsec = { 0, 100000000 };
	while ((nanosleep(&halfsec, &halfsec)) < 0)
		;
	return 0;
}

int mpu_cfg_write(struct mpu_dev *dev)
{
	if ((dev == NULL) || (dev->cfg == NULL)) 
		return -1;

	uint8_t	reg, val;
	int i;
	for (	len = dev->cfg->cfg[0][0], i = 1; i < dev->cfg->cfg[0][0]; i++) {
		reg = dev->cfg->cfg[i][0];
		val = dev->cfg->cfg[i][1];
		if ((mpu_comm_write_byte(dev, reg, val)) < 0)
			return -1;
		/* wait at least 10ms */
		struct timespec halfsec = { 0, 10000000 };
		while ((nanosleep(&halfsec, &halfsec)) < 0)
			;
	}

	return 0;
}

int mpu_cfg_validate(struct mpu_dev *dev)
{
	if ((dev == NULL) || (dev->cfg == NULL)) 
		return -1;

	uint8_t	reg, val;
	int i;
	for (	len = dev->cfg->cfg[0][0], i = 1; i < len; i++) {
		reg = dev->cfg->cfg[i][0];
		if ((mpu_comm_read_byte(dev, reg, &val)) < 0)
			return -1;
		if (val != dev->cfg->cfg[i][1])
			return -1;
	}

	return 0;
}

int mpu_cfg_get_val(struct mpu_dev *dev, const uint8_t reg, uint8_t *val)
{
	if ((dev == NULL) || (dev->cfg == NULL) || (val == NULL)) 
		return -1;
	
	uint8_t	len = dev->cfg->cfg[0][0];
	int i;

	for (i = 1; i < len; i++) {
		if (reg == dev->cfg->cfg[i][0]){
			*val = dev->cfg->cfg[i][1];
			return 0;
		}
	}
	return -1; /* reg not found */
}
int mpu_cfg_parse_PWR_MGMT_1(struct mpu_dev *dev)
{
	if ((dev == NULL) || (dev->cfg == NULL)) 
		return -1;
	
	uint8_t	val;
	if ((mpu_cfg_get_val(dev, PWR_MGMT_1, &val)) < 0)
		return -1; 
	switch ( val >> 3 ) {
	case 0x00 : dev->gfr =  250.0; break;
	case 0x01 : dev->gfr =  500.0; break;
	case 0x02 : dev->gfr = 1000.0; break;
	case 0x03 : dev->gfr = 2000.0; break;
	default : return -1; /* invalid value */
	}

	return 0;
}

int mpu_cfg_parse_FIFO_ENABLE(struct mpu_dev *dev)
{
	if ((dev == NULL) || (dev->cfg == NULL)) 
		return -1;
	
	uint8_t	val;
	if ((mpu_cfg_get_val(dev, FIFO_ENABLE, &val)) < 0)
		return -1; 

	/* CRUCIAL - raw[0] = buffered sensors + 1 */
	switch (val) {
	case 0x08 : dev->aen = 1; dev->gen = 0; dev->ten = 0; raw[0] = 4; break;
	case 0x78 : dev->aen = 1; dev->gen = 1; dev->ten = 0; raw[0] = 7; break;
	case 0xF8 : dev->aen = 1; dev->gen = 1; dev->ten = 1; raw[0] = 8; break;
	case 0x88 : dev->aen = 1; dev->gen = 0; dev->ten = 1; raw[0] = 5; break;
	case 0x70 : dev->aen = 0; dev->gen = 1; dev->ten = 0; raw[0] = 4; break;
	case 0xF0 : dev->aen = 0; dev->gen = 1; dev->ten = 1; raw[0] = 5; break;
	case 0x80 : dev->aen = 0; dev->gen = 0; dev->ten = 1; raw[0] = 2; break;
	case 0x00 : dev->aen = 0; dev->gen = 0; dev->ten = 0; raw[0] = 1; break;
	default :
		return -1; /* invalid value */
	}

	return 0;
}

int mpu_cfg_parse_ACCEL_CONFIG(struct mpu_dev *dev)
{
	if ((dev == NULL) || (dev->cfg == NULL)) 
		return -1;
	
	uint8_t	val;
	if ((mpu_cfg_get_val(dev, ACCEL_CONFIG, &val)) < 0)
		return -1; 
	switch ( val >> 3 ) {
	case 0x00 : dev->afr =  2.0; break;
	case 0x01 : dev->afr =  4.0; break;
	case 0x02 : dev->afr =  8.0; break;
	case 0x03 : dev->afr = 16.0; break;
	default : return -1; /* invalid value */
	} break;

	return 0;
}

int mpu_cfg_parse_GYRO_CONFIG(struct mpu_dev *dev)
{
	if ((dev == NULL) || (dev->cfg == NULL)) 
		return -1;
	
	uint8_t	val;
	if ((mpu_cfg_get_val(dev, GYRO_CONFIG, &val)) < 0)
		return -1; 
	switch ( val >> 3 ) {
	case 0x00 : dev->gfr =  250.0; break;
	case 0x01 : dev->gfr =  500.0; break;
	case 0x02 : dev->gfr = 1000.0; break;
	case 0x03 : dev->gfr = 2000.0; break;
	default : return -1; /* invalid value */
	}

	return 0;
}

int mpu_cfg_parse_CONFIG(struct mpu_dev *dev)
{
	if ((dev == NULL) || (dev->cfg == NULL)) 
		return -1;

	uint8_t	val;
	if ((mpu_cfg_get_val(dev, CONFIG, &val)) < 0)
		return -1; 
	switch (val) {
	case 0x00 : dev->abdw = 260.0; dev->adly =  0.0; dev->gbdw = 256.0; dev->gdly =  0.98; dev->gor = 8000; break;
	case 0x01 : dev->abdw = 184.0; dev->adly =  2.0; dev->gbdw = 188.0; dev->gdly =  1.90; dev->gor = 1000; break;
	case 0x02 : dev->abdw =  94.0; dev->adly =  3.0; dev->gbdw =  98.0; dev->gdly =  2.80; dev->gor = 1000; break;
	case 0x03 : dev->abdw =  44.0; dev->adly =  4.9; dev->gbdw =  42.0; dev->gdly =  4.00; dev->gor = 1000; break;
	case 0x04 : dev->abdw =  21.0; dev->adly =  8.5; dev->gbdw =  20.0; dev->gdly =  8.30; dev->gor = 1000; break;
	case 0x05 : dev->abdw =  10.0; dev->adly = 13.8; dev->gbdw =  10.0; dev->gdly = 13.40; dev->gor = 1000; break;
	case 0x06 : dev->abdw =   5.0; dev->adly = 19.0; dev->gbdw =   5.0; dev->gdly = 18.60; dev->gor = 1000; break;
	case 0x07 : dev->abdw =   0.0; dev->adly =  0.0; dev->gbdw =   0.0; dev->gdly =  0.00; dev->gor = 8000; break;
	default : return -1; /* invalid value */
	}

	return 0;
}


int mpu_cfg_parse_SMPLRT_DIV(struct mpu_dev *dev)
{
	if ((dev == NULL) || (dev->cfg == NULL)) 
		return -1;
	
	if (dev->gor == 0) /* must have gyro output rate */
		return -1;	

	uint8_t	val;
	if ((mpu_cfg_get_val(dev, CONFIG, &val)) < 0)
		return -1; 

	dev->sdiv = (val +1);
	dev->sr   = dev->gor / dev->sdiv;
	dev->st	  =	   1 / dev->sr;

	return 0;
}

int mpu_cfg_get_lbs(struct mpu_dev *dev)
{
	if (dev == NULL) 
		return -1;
	/*
	 * find LBS - Lowest Bit Lensitivity
	 * 16 bits signed data range
	 * max: 2⁽¹⁶⁻¹⁾   = +32768;
	 * min: 2⁽¹⁶⁻¹⁾-1 = -32767;
	 */
	if ((dev->afr == 0) || (dev->gfr == 0))
		return -1;
	dev->albs = 32768.0/dev->afr;
	dev->glbs = 32768.0/dev->gfr;

	return 0;
}

/*
 * Handles the setup of "scale" array
 *
 * Each entry is detemined by the range
 * settings stored in the config table
 */
int mpu_cfg_get_scaling(struct mpudev *dev)
{
	if ((dev == NULL) || (dev->dat == NULL)) 
		return -1;

	int i, cnt;

	cnt = 1;
	if (dev->aen) {
		if (dev->albs == 0) /* division by zero */
			return -1;
		for (i = 0; i < 3; i++, cnt++) {
			dev->dat->scl[cnt] = 1/dev->albs;
		}
	}

	if (dev->ten) {
		dev->dat->scale[cnt]=1/340;
		cnt++;
	}
	if (dev->gen) {
		if (dev->glbs == 0) /* division by zero */
			return -1;
		for (i = 0; i < 3; i++, cnt++) {
			dev->dat->scl[cnt] = 1/dev->glbs;
		}
	}

	if (cnt == 1)
		return -1;	/* no sensor enabled */

	return 0;
}

int mpu_cfg_get_pointers(struct mpu_dev *dev)
{
	if ((dev == NULL) || (dev->dat == NULL) || (dev->cal == NULL)) 
		return -1;

	/* Associate data with meaningful names */
	int count = 1;
	if (dev->aen) {
		dev->Ax  = &dev->dat->dat[count][0];
		dev->Ax2 = &dev->dat->squ[count];
		dev->Axo = &dev->cal->off[count];
		dev->Axg = &dev->cal->gai[count];
	dev->Axd = &dev->cal->dri[count];
		dev->Axm = &dev->dat->mea[count];
		dev->Axv = &dev->dat->var[count];
		count++;
		dev->Ay  = &dev->dat->dat[count][0];
		dev->Ay2 = &dev->dat->squ[count];
		dev->Ayo = &dev->cal->off[count];
		dev->Ayg = &dev->cal->gai[count];
		dev->Ayd = &dev->cal->dri[count];
		dev->Aym = &dev->dat->mea[count];
		dev->Ayv = &dev->dat->var[count];
		count++;
		dev->Az  = &dev->dat->dat[count][0];
		dev->Az2 = &dev->dat->squ[count];
		dev->Azo = &dev->cal->off[count];
		dev->Azg = &dev->cal->gai[count];
		dev->Azd = &dev->cal->dri[count];
		dev->Azm = &dev->dat->mea[count];
		dev->Azv = &dev->dat->var[count];
		count++;
	}
	if (dev->ten)	{
		dev->t = &dev->dat->dat[count][0];
		dev->to = &dev->cal->off[count];
		dev->tg = &dev->cal->gai[count];
		dev->td = &dev->cal->dri[count];
		dev->tm = &dev->dat->mea[count];
		dev->tv = &dev->dat->var[count];
		count++;
	}
	if (dev->gen)	{
		dev->Gx  = &dev->dat->dat[count][0];
		dev->Gx2 = &dev->dat->squ[count];
		dev->Gxo = &dev->cal->off[count];
		dev->Gxg = &dev->cal->gai[count];
		dev->Gxd = &dev->cal->dri[count];
		dev->Gxm = &dev->dat->mea[count];
		dev->Gxv = &dev->dat->var[count];
		count++;
		dev->Gy  = &dev->dat->dat[count][0];
		dev->Gy2 = &dev->dat->squ[count];
		dev->Gyo = &dev->cal->off[count];
		dev->Gyg = &dev->cal->gai[count];
		dev->Gyd = &dev->cal->dri[count];
		dev->Gym = &dev->dat->mea[count];
		dev->Gyv = &dev->dat->var[count];
		count++;
		dev->Gz  = &dev->dat->dat[count][0];
		dev->Gz2 = &dev->dat->squ[count];
		dev->Gzo = &dev->cal->off[count];
		dev->Gzg = &dev->cal->gai[count];
		dev->Gzd = &dev->cal->dri[count];
		dev->Gzm = &dev->dat->mea[count];
		dev->Gzv = &dev->dat->var[count];
	}
	if (count == 1)	/* all sensors disabled */
		return -1;

	return 0;
}


int mpu_cfg_reset_pointers(struct mpu_dev *dev)
{
	if (dev == NULL) 
		return -1;
	/* make everything point to null */
	dev->Ax  = NULL;
	dev->Ax2 = NULL;
	dev->Axo = NULL;
	dev->Axg = NULL;
	dev->Axd = NULL;
	dev->Axm = NULL;
	dev->Axv = NULL;
	dev->Ay  = NULL;
	dev->Ay2 = NULL;
	dev->Ayo = NULL;
	dev->Ayg = NULL;
	dev->Ayd = NULL;
	dev->Aym = NULL;
	dev->Ayv = NULL;
	dev->Az  = NULL;
	dev->Az2 = NULL;
	dev->Azo = NULL;
	dev->Azg = NULL;
	dev->Azd = NULL;
	dev->Azm = NULL;
	dev->Azv = NULL;
	dev->t	 = NULL;
	dev->to	 = NULL;
	dev->tg	 = NULL;
	dev->td	 = NULL;
	dev->tm	 = NULL;
	dev->tv	 = NULL;
	dev->Gx  = NULL;
	dev->Gx2 = NULL;
	dev->Gxo = NULL;
	dev->Gxg = NULL;
	dev->Gxd = NULL;
	dev->Gxm = NULL;
	dev->Gxv = NULL;
	dev->Gy  = NULL;
	dev->Gy2 = NULL;
	dev->Gyo = NULL;
	dev->Gyg = NULL;
	dev->Gyd = NULL;
	dev->Gym = NULL;
	dev->Gyv = NULL;
	dev->Gz  = NULL;
	dev->Gz2 = NULL;
	dev->Gzo = NULL;
	dev->Gzg = NULL;
	dev->Gzd = NULL;
	dev->Gzm = NULL;
	dev->Gzv = NULL;
	return 0;
}
int mpu_cfg_parse(struct mpu_dev *dev)
{
	if ((mpu_cfg_parse_FIFO_ENABLE(dev)) < 0 )
	         return -1;
	if ((mpu_cfg_parse_ACCEL_CONFIG(dev)) < 0 )
	         return -1;
	if ((mpu_cfg_parse_GYRO_CONFIG(dev)) < 0 )
	         return -1;
	if ((mpu_cfg_parse_CONFIG(dev)) < 0 )
	         return -1;
	if ((mpu_cfg_parse_SMPLRT_DIV(dev)) < 0)
		return -1;

	if ((mpu_cfg_get_lbs(dev)) < 0)
		return -1;

	if ((mpu_cfg_get_scaling(dev)) < 0)
		return -1;


	// TODO: Why?????
	for (int i=1; i < raw[0]; i++){
		data[i][1] = data[i][0];
		data[i][0] = (raw[i]*scale[i]);
	}
}

int mpu_cal_reset(struct mpu_dev *dev)
{
	if ((dev == NULL) || (dev->cal == NULL))
		return -1;

	struct mpu_cal unc = {
		.gra = 1,
		.gai = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
	};
	memcpy((void *)dev->cal, (void *)&unc, sizeof(struct mpu_cal));

	return 0;
}

/*
 * Part 1. UAPI
 * On a Raspberry Pi, path should be "/dev/i2c-1/"
 * 	The default  address for MPU6050 is 0x68
 * 	The device must be registered in the bus
 * 	Probe channel (WHOAMI)
 */
int mpu_init(	const char * restrict path,
		const uint8_t address,
		struct mpu_dev restrict *mpudev);
{
	/* check parameters */
	if 	((path == NULL) || 		/* null pointer */
		(address == (uint8_t)0x00))	/* invalid address */
		return -1;

	size_t pathlen = strlen(path);
	if (pathlen < 6)			/* invalid path */
		return -1;

	/* allocate memory for new struct */
	struct mpu_dev *dev = NULL;

	dev = calloc(1, sizeof(struct mpu_dev));
	if (dev == NULL) {
		free(dev);
		return -1;
	}
	dev->bus = NULL;
	dev->bus = calloc(1, sizeof(int));
	dev->cfg = NULL;
	dev->cfg = calloc(1, sizeof(struct mpu_cfg));
	dev->cal = NULL;
	dev->cal = calloc(1, sizeof(struct mpu_cal));
	dev->dat = NULL;
	dev->dat = calloc(1, sizeof(struct mpu_dat));

	/* from now on, all errors go to mpu_init_err for deallocation */
	if ((dev->bus == NULL) || (dev->cfg == NULL) || (dev->dat == NULL) || (dev->cal == NULL))
		goto mpu_new_err;

	/* set meaningful pointers to NULL for later assignment */
	if ((mpu_cfg_reset_pointers(dev)) < 0);
		goto mpu_new_err;

	/* ensure device is reachable */
	dev->addr = address;
	if ((*(dev->bus) = open(path, O_RDWR)) < 0) /* path unavailable */
		goto mpu_new_err;
	if ((ioctl(dev->bus, I2C_SLAVE, dev->addr)) < 0) /* bus error */
		goto mpu_new_err;

	uint8_t	checkaddr;
	if ((mpu_comm_read_byte(dev->bus, WHO_AM_I, &checkaddr)) < 0)
		goto mpu_new_err;
	if (bus->addr != checkaddr) /* address mismatch */
		goto mpu_new_err;

	/* ensure device is not sleeping */
	if ((mpu_ctl_wake(dev)) < 0)
		goto mpu_new_err;

	if ((mpu_ctl_set_CLKSEL(dev, 0x03)) < 0) /* PLL with Z axis gyro reference */
		goto mpu_new_err;
	
	/* ensure device has default configuration */
	//TODO make a separate function, 
	//TODO mate defcfg private
	//TODO fix sizeof paramater to check boundary
	memcpy((void *)dev->cfg, (void *)&mpu6050_defcfg, sizeof(struct mpu_cfg));

	/* Must write config to device registers */
	if ((mpu_cfg_write(dev)) < 0)
		goto mpu_new_err;
	/* Must ensure the config is written to the device */
	if ((mpu_cfg_validate(dev)) < 0)
		goto mpu_new_err;
	/* Must parse config to readable variables */
	if ((mpu_cfg_parse(dev)) < 0)
		goto mpu_new_err;

	if ((mpu_cfg_get_pointers(dev)) < 0)
		goto mpu_new_err;

	/* Must have a default calibration */
	if ((mpu_cal_reset(dev)) < 0)
		goto mpu_new_err;


	/* everything ok, assign built struct to te parameter */
	mpudev = dev;

	return 0;

mpu_new_err:
	free(dev->bus);
	free(dev->cfg);
	free(dev->dat);
	free(dev->cal);
	free(dev);

	return -1;

};
