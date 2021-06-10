#include <stdlib.h>	   /* for malloc(), free(), exit() */
#include <stdio.h>	   /* for printf(), scanf(), fopen(), perror() */
#include <string.h>	   /* for memcpy() */
#include <unistd.h>	   /* for close(), write(), getopt(), size_t */
#include <inttypes.h>	   /* for uint8_t, uint16_t, etc */
#include <stdint.h>	   /* for uint8_t, uint16_t, etc */
#include <stddef.h>	   /* for size_t */
#include <stdbool.h>	   /* for bool */
#include <fcntl.h>	   /* for open() */
#include <sys/stat.h>	   /* for open() */
#include <sys/types.h>	   /* for open() */
#include <sys/ioctl.h>	   /* for ioctl() */
#include <linux/types.h>   /* for __u8, __s32 */
#include <i2c/smbus.h> 	   /* for i2c_smbus_x */
#include <linux/i2c.h> 	   /* for i2c_smbus_x */
#include <linux/i2c-dev.h> /* for i2c_smbus_x */
#include "dev_mpu_header.h"
#include <stdio.h>
#include <math.h>

	
int mpu_init(	const char * path,
		const uint8_t address,
		struct mpu_dev **mpudev);

int mpu_destroy(struct mpu_dev * mpudev);

void mpu_print_data(struct mpu_dev *dev, int times);

int mpu_ctl_selftest(struct mpu_dev * dev);
int mpu_ctl_selftest_enable_accel(struct mpu_dev *dev);
int mpu_ctl_selftest_enable_gyro(struct mpu_dev *dev);
int mpu_ctl_selftest_disable_accel(struct mpu_dev *dev);
int mpu_ctl_selftest_disable_gyro(struct mpu_dev *dev);

int mpu_ctl_fifo_enable		(struct mpu_dev *dev);
int mpu_ctl_fifo_enable_accel	(struct mpu_dev *dev);
int mpu_ctl_fifo_enable_gyro	(struct mpu_dev *dev);
int mpu_ctl_fifo_enable_temp	(struct mpu_dev *dev);
int mpu_ctl_fifo_disable	(struct mpu_dev *dev);
int mpu_ctl_fifo_disable_accel	(struct mpu_dev *dev);
int mpu_ctl_fifo_disable_gyro	(struct mpu_dev *dev);
int mpu_ctl_fifo_disable_temp	(struct mpu_dev *dev);
int mpu_ctl_fifo_count		(struct mpu_dev *dev, int *count);
int mpu_ctl_fifo_data		(struct mpu_dev *dev);
int mpu_ctl_fifo_flush		(struct mpu_dev *dev);
int mpu_ctl_fifo_reset		(struct mpu_dev *dev);

int mpu_ctl_i2c_mst_reset	(struct mpu_dev *dev);
int mpu_ctl_sig_cond_reset	(struct mpu_dev *dev);


int mpu_ctl_wake(struct mpu_dev *  dev);

int mpu_ctl_samplerate( struct mpu_dev * dev,
			unsigned int rate_hz);

int mpu_ctl_dlpf(struct mpu_dev * dev,
	       	 unsigned int dlpf);

int mpu_ctl_accel_range( struct mpu_dev * dev,
			unsigned int range);

int mpu_ctl_gyro_range( struct mpu_dev * dev,
			unsigned int range);

int mpu_ctl_temperature(struct mpu_dev * dev,
			bool temp_on);

int mpu_ctl_clocksource(struct mpu_dev * dev,
			mpu_reg_t clksel);

int mpu_read_byte(struct mpu_dev * const dev,
		const mpu_reg_t reg, 		/* device register */
		mpu_reg_t * val);		/* value destination */

int mpu_read_word(struct mpu_dev * const dev,
		const mpu_reg_t reg, 		/* device register */
		mpu_word_t * val);		/* value destination */

int mpu_read_data(struct mpu_dev * const dev,
		const mpu_reg_t reg, 		/* device register */
		int16_t * val);			/* value destination */

int mpu_write_byte(struct mpu_dev * const dev,
		const mpu_reg_t reg, 		/* device register */
		const mpu_reg_t val);		/* value to write */

int mpu_write_word(struct mpu_dev * const dev,
		const mpu_reg_t reg, 		/* device register */
		const mpu_word_t val);		/* value to write */
/* helpers - internal use only */
static int mpu_dev_bind(const char * path, 
			const mpu_reg_t address,
			struct mpu_dev * dev);

static int mpu_dev_allocate(struct mpu_dev **dev);
static int mpu_cfg_set(struct mpu_dev * dev);
static int mpu_dat_set(struct mpu_dev * dev);

static int mpu_cfg_reset(struct mpu_dev * dev);
static int mpu_dat_reset(struct mpu_dev * dev);
static int mpu_cal_reset(struct mpu_dev * dev);

static int mpu_cfg_set_CLKSEL(struct mpu_dev * dev,
			      mpu_reg_t clksel);


static int mpu_cfg_get_val(struct mpu_dev * dev,
			   const mpu_reg_t reg,
			   mpu_reg_t * val);

static int mpu_cfg_set_val(struct mpu_dev * dev,
			   const mpu_reg_t reg,
			   const mpu_reg_t val);

static int mpu_cfg_write(	struct mpu_dev * dev);
static int mpu_cfg_validate(	struct mpu_dev * dev);
static int mpu_cfg_parse(	struct mpu_dev * dev);
static int mpu_cfg_parse_PWR_MGMT(	struct mpu_dev * dev);
static int mpu_cfg_parse_CONFIG(	struct mpu_dev * dev);
static int mpu_cfg_parse_SMPLRT_DIV(	struct mpu_dev * dev);
static int mpu_cfg_parse_ACCEL_CONFIG(	struct mpu_dev * dev);
static int mpu_cfg_parse_GYRO_CONFIG(	struct mpu_dev * dev);
static int mpu_cfg_parse_USER_CTRL(	struct mpu_dev * dev);
static int mpu_cfg_parse_FIFO_EN(	struct mpu_dev * dev);
static int mpu_cfg_parse_INT_ENABLE(	struct mpu_dev * dev);
static int mpu_cfg_parse_INT_PIN_CFG(	struct mpu_dev * dev);

static int mpu_fifo_data(struct mpu_dev *dev, int16_t *data);

#define MPUDEV_IS_NULL(dev)	((NULL == (dev)) ||      \
				 (NULL == (dev)->dat) || \
				 (NULL == (dev)->cfg) || \
				 (NULL == (dev)->bus) || \
				 (NULL == (dev)->cal))

#define MPUDEV_NOT_NULL(dev)	((NULL != (dev)) &&      \
				 (NULL != (dev)->dat) && \
				 (NULL != (dev)->cfg) && \
				 (NULL != (dev)->bus) && \
				 (NULL != (dev)->cal))

int mpu_read_byte(struct mpu_dev * const dev,
		const mpu_reg_t reg,
		mpu_reg_t *val)
{
	if ((NULL == dev) || (NULL == dev->bus))
		return -1;

	__s32 res = i2c_smbus_read_byte_data(*(dev->bus), reg);

	if (res < 0) /* read failed - bus error */
		return -1;

	*val = (mpu_reg_t) res;
	return 0;
	
}

int mpu_write_byte(struct mpu_dev * const dev,
		const mpu_reg_t reg,
		const mpu_reg_t val)
{
	if ((NULL == dev) || (NULL == dev->bus))
		return -1;

	__s32 res = i2c_smbus_write_byte_data(*(dev->bus), reg, val);

	if (res < 0) /* read failed - bus error */
		return -1;

	return 0;
	
}

int mpu_read_word(struct mpu_dev * const dev,
		const mpu_reg_t reg,
		mpu_word_t *val)
{
	if ((NULL == dev) || (NULL == dev->bus))
		return -1;

	__s32 res = i2c_smbus_read_word_data(*(dev->bus), reg);

	if (res < 0) /* write byte failed - bus error */
		return -1;

	*val = (mpu_word_t) res;
	return 0;
	
}

int mpu_write_word(struct mpu_dev * const dev,
		const mpu_reg_t reg,
		const mpu_word_t val)
{
	if ((NULL == dev) || (NULL == dev->bus))
		return -1;

	__s32 res = i2c_smbus_write_word_data(*(dev->bus), reg, val);

	if (res < 0) /* write word failed - bus error */
		return -1;

	return 0;
	
}

/*
 * USAGE:
 * Create an empty
 *
 * Call mpu_init()
 * 	Use path "/dev/i2c-1/" (default)
 * 	Use address 0x68 (default)
 * 	Use mpudev (mpudev must be NULL)
 *
 *	 Success( 0) mpudev ready.
 *	 Failure(-1) nothing changed.
 */
int mpu_init(	const char * const restrict path,
		const mpu_reg_t address,
		struct mpu_dev ** mpudev)
{
	
	if (*mpudev != NULL) /* device not empty */
		return -1;
	
	if ((address == (mpu_reg_t)0x00)) /* invalid address */
		return -1;
	
	size_t pathlen = strlen(path);
	if ((path == NULL) || (pathlen < 6)) /* invalid path */
		return -1;

	
	struct mpu_dev *dev = NULL;
	if ((mpu_dev_allocate(&dev)) < 0) /* no memory allocated */
		return -1;

	
	if ((mpu_dev_bind(path, address, dev)) < 0) /* could't bind */
		goto mpu_init_error;
	
	if ((mpu_ctl_wake(dev)) < 0) /* wake up failed */
		goto mpu_init_error;
	
	if ((mpu_cfg_set_CLKSEL(dev, CLKSEL_3)) < 0) 
		goto mpu_init_error;
	
	
	if ((mpu_dat_reset(dev)) < 0) /* clean data pointers */
		goto mpu_init_error;

	
	if ((mpu_cfg_reset(dev)) < 0) /* assign default config */
		goto mpu_init_error;

	
	if ((mpu_cal_reset(dev)) < 0) /* assign unity gain, zero errors */
		goto mpu_init_error;

	
	if ((mpu_dat_set(dev)) < 0) /* assign data pointers */
		goto mpu_init_error;

	if ((mpu_read_byte(dev, PROD_ID, &(dev->prod_id))) < 0) /* get product id */
		goto mpu_init_error;
	
	*mpudev = dev; /* success */
	return 0;

mpu_init_error:
	
	if (dev->bus != NULL) {
		if ((close(*(dev->bus))) < 0) /* close failed */
			exit(EXIT_FAILURE);
	}

	
	if ((mpu_destroy(dev)) < 0) /* cleanup failed, check for bugs */
		exit(EXIT_FAILURE);

	return -1;
};

int mpu_destroy(struct mpu_dev * dev)
{
	
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;
	
	mpu_dat_reset(dev);
	
	free(dev->cal); dev->cal = NULL;
	free(dev->dat); dev->dat = NULL;
	free(dev->cfg); dev->cfg = NULL;
	free(dev->bus); dev->bus = NULL;
	free(dev); dev = NULL;
	
	return 0;
}

int mpu_ctl_wake(struct mpu_dev * dev)
{
	
	if ((NULL == dev) || (NULL == dev->bus)) 
		return -1;
	
	mpu_reg_t val;
	if ((mpu_cfg_get_val(dev, PWR_MGMT_1, &val)) < 0) { /* unconfigured */
		if (mpu_write_byte(dev, PWR_MGMT_1, 0x00) < 0) { /* write failed */
			return -1;
		} else {
			return 0;
		}
	}

	/* clear DEVICE_RESET, SLEEP and CYCLE bits */
	val &= (mpu_reg_t) ~(DEVICE_RESET_BIT | SLEEP_BIT | CYCLE_BIT);
	if ((mpu_cfg_set_val(dev, PWR_MGMT_1, val)) < 0)
		return -1;

	/* cleaer all STDBY bits */
	if ((mpu_cfg_set_val(dev, PWR_MGMT_2, 0x00)) < 0)
		return -1;

	if ((mpu_cfg_set(dev)) < 0)
		return -1;
	if ((mpu_dat_set(dev)) < 0)
		return -1;
	
	return 0;
}

static int mpu_cfg_set_CLKSEL(struct mpu_dev * dev, mpu_reg_t clksel)
{
	
	if ((NULL == dev) || (NULL == dev->bus)) 
		return -1;
	
	if (clksel > 7) /* out of range  */
		return -1;

	
	switch(clksel) {
		case 0: break; /* Internal 8 Mhz oscillator */
		case 1: break; /* PLL with X axis gyroscope */
		case 2: break; /* PLL with Y axis gyroscope */
		case 3: break; /* PLL with Z axis gyroscope */
		default: /* not supported */
			return -1;
	}
	
	
	mpu_reg_t val;
	if ((mpu_read_byte(dev, PWR_MGMT_1, &val)) < 0)
		return -1;
	
	val  &= (~0x07u); /* mask CLK_SEL bits */
	
	val  |= clksel;	 /* set  CLK_SEL bits */

	if ((mpu_write_byte(dev, PWR_MGMT_1, val)) < 0)
		return -1;

	return 0;
}

int mpu_ctl_dlpf(struct mpu_dev * dev, unsigned int dlpf)
{
	
	if ((NULL == dev) || (NULL == dev->bus)) 
		return -1;
	if (dlpf > 7) { /* invalid dlpf_cfg value */
		return -1;
	}

	/* get DLPF_CFG value */
	mpu_reg_t val;
	if ((mpu_read_byte(dev, CONFIG, &val)) < 0) {
		return -1;
	}
	/* break circular dependencies */
	if((val & DLPF_CFG_BIT) == (dlpf & DLPF_CFG_BIT)) {
		return 9;
	}
	
	unsigned int old_rate_hz = dev->sr;
	
	if ((mpu_cfg_get_val(dev, CONFIG, &val)) < 0)
		return -1;
	val &= ~(DLPF_CFG_BIT);
	val |= (dlpf & DLPF_CFG_BIT);

	if ((mpu_cfg_set_val(dev, CONFIG, val)) < 0)
		return -1;
	if ((mpu_cfg_set(dev)) < 0)
		return -1;
	if ((mpu_dat_set(dev)) < 0)
		return -1;

	unsigned int new_rate_hz = dev->sr;
	if (old_rate_hz != new_rate_hz) {
		if ((mpu_ctl_samplerate(dev, old_rate_hz)) < 0)
			return -1;
	}
	
	return 0;
}

int mpu_ctl_samplerate(struct mpu_dev * dev, unsigned int rate_hz)
{
	
	if ((NULL == dev) || (NULL == dev->bus)) 
		return -1;
	
	/* break circular dependencies */
	unsigned int old_rate_hz = dev->sr;
	if( rate_hz == old_rate_hz ) {
		return 9;
	}

	switch (rate_hz) { 		/* base 8kHz  / 1kHz	*/
		case 100: break;	/*	  80 	10	*/
		case  50: break;	/*	 160 	20	*/
		default: return -1;	/* rate not supported	*/
	}
	
	/* get DLPF_CFG value */
	mpu_reg_t val;
	if ((mpu_read_byte(dev, CONFIG, &val)) < 0)
		return -1;
	
	unsigned int fs_base = (val & DLPF_CFG_BIT) ? 1000 : 8000;

	unsigned int divider = (fs_base / rate_hz);
	
	val = (mpu_reg_t) (divider - 1); /* rate_hz = fs_base / (1 + SMPLRT_DIV) */
	
	if ((mpu_cfg_set_val(dev, SMPLRT_DIV, val)) < 0)
		return -1;
	if ((mpu_cfg_set(dev)) < 0)
		return -1;
	if ((mpu_dat_set(dev)) < 0)
		return -1;
	
	return 0;
}

int mpu_ctl_accel_range(struct mpu_dev * dev, unsigned int range)
{
	
	if ((NULL == dev) || (NULL == dev->bus)) 
		return -1;
	
	mpu_reg_t afs_sel = 0;
	switch (range) { 	
		case  2: afs_sel = AFS_SEL_0; break;
		case  4: afs_sel = AFS_SEL_1; break;
		case  8: afs_sel = AFS_SEL_2; break;
		case 16: afs_sel = AFS_SEL_3; break;
		default: return -1;	/* invalid range */
	}
	
	mpu_reg_t reg = ACCEL_CONFIG;
	mpu_reg_t val;
	
	if ((mpu_read_byte(dev, reg, &val)) < 0)
		return -1;
	
	val &= (~AFS_SEL_BIT);  /* mask bits */
	
	val |= afs_sel; 	/* set bits */
	
	if (mpu_cfg_set_val(dev, reg, val) < 0)
		return -1;

	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;
	if (mpu_ctl_fifo_flush(dev) < 0)
	       return -1;	

	return 0;
}

int mpu_ctl_gyro_range(struct mpu_dev * dev, unsigned int range)
{
	
	if ((NULL == dev) || (NULL == dev->bus)) 
		return -1;
	
	mpu_reg_t fs_sel = 0;
	switch (range) { 	
		case  250: fs_sel = FS_SEL_0; break;
		case  500: fs_sel = FS_SEL_1; break;
		case 1000: fs_sel = FS_SEL_2; break;
		case 2000: fs_sel = FS_SEL_3; break;
		default: return -1;	/* invalid range */
	}
	
	mpu_reg_t val;
	if ((mpu_cfg_get_val(dev, GYRO_CONFIG, &val)) < 0)
		return -1;

	val &= (~FS_SEL_BIT);  /* mask bits */
	val |= fs_sel; 		/* set bits */

	if ((mpu_cfg_set_val(dev, GYRO_CONFIG, val)) < 0)
		return -1;

	if ((mpu_cfg_set(dev)) < 0)
		return -1;
	if ((mpu_dat_set(dev)) < 0)
		return -1;
	if (mpu_ctl_fifo_flush(dev) < 0)
	       return -1;	
	
	return 0;
}

int mpu_ctl_temperature(struct mpu_dev * dev, bool temp_on)
{
	if ((NULL == dev) || (NULL == dev->bus)) 
		return -1;

	mpu_reg_t val;
	if ((mpu_cfg_get_val(dev, PWR_MGMT_1, &val)) < 0)
		return -1;
	
	if (temp_on)
		val &= (~TEMP_DIS_BIT); /* temperature sensor on */
	else
		val |= TEMP_DIS_BIT;    /* temperature sensor off */

	
	if ((mpu_cfg_set_val(dev, PWR_MGMT_1, val)) < 0)
		return -1;
	
	if ((mpu_cfg_set(dev)) < 0)
		return -1;
	if ((mpu_dat_set(dev)) < 0)
		return -1;
	if (mpu_ctl_fifo_flush(dev) < 0)
	       return -1;	

	return 0;
}

int mpu_ctl_clocksource(struct mpu_dev * dev, mpu_reg_t clksel)
{
	
	if ((NULL == dev) || (NULL == dev->bus)) 
		return -1;

	
	switch(clksel) {
		case CLKSEL_0: break; /* Internal 8 Mhz oscillator */
		case CLKSEL_1: break; /* PLL with X axis gyroscope */
		case CLKSEL_2: break; /* PLL with Y axis gyroscope */
		case CLKSEL_3: break; /* PLL with Z axis gyroscope */
		default: /* not supported */
			return -1;
	}

	
	mpu_reg_t val;
	if ((mpu_cfg_get_val(dev, PWR_MGMT_1, &val)) < 0)
		return -1;

	val  &= (~CLKSEL_BIT); /* mask CLK_SEL bits */
	val  |= clksel;	 /* set  CLK_SEL bits */

	
	if ((mpu_cfg_set_val(dev, PWR_MGMT_1, val)) < 0)
		return -1;

	
	if ((mpu_cfg_set(dev)) < 0)
		return -1;
	if ((mpu_dat_set(dev)) < 0)
		return -1;

	
	return 0;
}

static int mpu_cfg_reset(struct mpu_dev * dev)
{
	
	if ((NULL == dev) || (NULL == dev->bus) || (NULL == dev->cfg))
		return -1;

	
	memcpy((void *)dev->cfg, (void *)&mpu6050_defcfg, sizeof(struct mpu_cfg));

	
	if ((mpu_cfg_set(dev)) < 0) /* couldn't set config */
		return -1;

	
	return 0;
}

static int mpu_cfg_set(struct mpu_dev * dev)
{
	
	if ((NULL == dev) || (NULL == dev->bus) || (NULL == dev->cfg))
		return -1;

	/* write config to device registers */
	if ((mpu_cfg_write(dev)) < 0)
		return -1;

	/* ensure device is configured */
	if ((mpu_cfg_validate(dev)) < 0)
		return -1;

	/* fill device structure */
	if ((mpu_cfg_parse(dev)) < 0)
		return -1;

	return 0;
}

static int mpu_cfg_write(struct mpu_dev * dev)
{
	
	if ((NULL == dev) || (NULL == dev->cfg)) 
		return -1;

	mpu_reg_t reg; /* device register address */
	mpu_reg_t val; /* device register value */

	unsigned int i = 0;
	unsigned int len = sizeof(dev->cfg->cfg)/sizeof(dev->cfg->cfg[0]);
	
	for (i = 0; i < len; i++) {
		reg = dev->cfg->cfg[i][0];
		val = dev->cfg->cfg[i][1];
		
		if ((mpu_write_byte(dev, reg, val)) < 0) /* write error */
			return -1;
	}
	

	return 0;
}

static int mpu_cfg_validate(struct mpu_dev * dev)
{
	if ((NULL == dev) || (NULL == dev->cfg)) 
		return -1;

	mpu_reg_t dev_val; /* device register value */
	
	unsigned int i = 1;
	unsigned int len = sizeof(dev->cfg->cfg)/sizeof(dev->cfg->cfg[0]);
	
	for (i = 0; i < len; i++) {
		mpu_reg_t reg 	  = dev->cfg->cfg[i][0];
		mpu_reg_t cfg_val = dev->cfg->cfg[i][1];
		
		if ((mpu_read_byte(dev, reg, &dev_val)) < 0) /* read error */
			return -1;

		
		if (cfg_val != dev_val) /* value mismatch */
			return -1;
	}

	
	return 0;
}

static int mpu_cfg_get_val(struct mpu_dev * dev, const mpu_reg_t reg, mpu_reg_t *val)
{
	
	if ((NULL == dev) || (NULL == dev->cfg) || (val == NULL)) 
		return -1;

	if (reg == 0) /* register 0 invalid */
		return -1;
	
	unsigned int i   = 0;
	unsigned int len = sizeof(dev->cfg->cfg)/sizeof(dev->cfg->cfg[0]);

	
	for (i = 0; i < len; i++) {
		if (0 == dev->cfg->cfg[i][0]) 	/* register 0 invalid */
			continue;
		if (reg == dev->cfg->cfg[i][0]) {/* seek register */
			
			*val = dev->cfg->cfg[i][1];
			return 0;
		}
		
	}
 
	
	return -1; /* reg not found */

}

static int mpu_cfg_set_val(struct mpu_dev * dev, const mpu_reg_t reg, const mpu_reg_t val)
{
	
	if ((NULL == dev) || (NULL == dev->cfg)) 
		return -1;
	
	if (reg == 0) /* register 0 invalid */
		return -1;

	unsigned int i = 0;
	unsigned int len = sizeof(dev->cfg->cfg)/sizeof(dev->cfg->cfg[0]);
	
	for (i = 0; i < len; i++) {
		if (0 == dev->cfg->cfg[i][0]) 	/* register 0 invalid */
			continue;
		
		if(reg == dev->cfg->cfg[i][0]) { /* seek register */
			dev->cfg->cfg[i][1] = val;
			
			return 0;
		}
	}

	
	return -1; /* reg not found */
}

static int mpu_cfg_parse_PWR_MGMT(struct mpu_dev * dev)
{
	
	if ((NULL == dev) || (NULL == dev->bus)) 
		return -1;

	mpu_reg_t val1;
	mpu_reg_t val2;
	
	if ((mpu_cfg_get_val(dev, PWR_MGMT_1, &val1)) < 0)
		return -1;
	
	if ((mpu_cfg_get_val(dev, PWR_MGMT_2, &val2)) < 0)
		return -1;
	
	if (val1 & DEVICE_RESET_BIT) /* must not be set */
		return -1;

	if (val1 & SLEEP_BIT)	 { dev->cfg->sleep = true;    }
	if (val1 & CYCLE_BIT)	 { dev->cfg->cycle = true;    }
	if (val1 & TEMP_DIS_BIT) { dev->cfg->temp_dis = true; }
	if (val2 & STDBY_XA_BIT) { dev->cfg->stdby_xa = true; }
	if (val2 & STDBY_YA_BIT) { dev->cfg->stdby_ya = true; }
	if (val2 & STDBY_ZA_BIT) { dev->cfg->stdby_za = true; }
	if (val2 & STDBY_XG_BIT) { dev->cfg->stdby_xg = true; }
	if (val2 & STDBY_YG_BIT) { dev->cfg->stdby_yg = true; }
	if (val2 & STDBY_ZG_BIT) { dev->cfg->stdby_zg = true; }

	double	wake_freq;
	int lp_wake_ctrl = (val2 & 0xC0u) >> 6;

	
	if (!(dev->cfg->cycle)) {
		wake_freq = 0;
	} else {
		switch(lp_wake_ctrl) {
		case 0 : wake_freq = 1.2; break;
		case 1 : wake_freq = 5;	  break;
		case 2 : wake_freq = 20;  break;
		case 3 : wake_freq = 40;  break;
		}
	}
	
	dev->wake_freq	= wake_freq;
	
	return 0;
}

static int mpu_cfg_parse_USER_CTRL(struct mpu_dev * dev)
{
	
	if ((NULL == dev) || (NULL == dev->bus)) 
		return -1;

	mpu_reg_t val;
	if ((mpu_cfg_get_val(dev, USER_CTRL, &val)) < 0)
		return -1;
	if (val & FIFO_RESET_BIT) {
		printf("FIFO_RESET_BIT set \n");
		return -1;
	}
	if (val & I2C_MST_RESET_BIT) {
		printf("I2C_MST_RESET_BIT set\n");
		return -1;

	}
	if (val & SIG_COND_RESET_BIT) {
		printf("SIG_COND_RESET_BIT set\n");
		return -1;
	}


	if(val & I2C_IF_DIS_BIT) /* this is valid but will disable i2c communication */
		return -1;
	
	dev->cfg->fifo_en	= (val & FIFO_EN_BIT)	 ? true : false;
	dev->cfg->i2c_mst_en	= (val & I2C_MST_EN_BIT) ? true : false; 
	dev->cfg->i2c_if_dis	= (val & I2C_IF_DIS_BIT) ? true : false;
	
	return 0;
}

static int mpu_cfg_parse_FIFO_EN(struct mpu_dev * dev)
{
	
	if ((NULL == dev) || (NULL == dev->cfg)) 
		return -1;
	
	mpu_reg_t val;
	if ((mpu_cfg_get_val(dev, FIFO_EN, &val)) < 0)
		return -1; 

	bool temp_fifo_en  = false;
	bool xg_fifo_en    = false;
	bool yg_fifo_en    = false;
	bool zg_fifo_en    = false;
	bool accel_fifo_en = false;
	bool slv2_fifo_en  = false;
	bool slv1_fifo_en  = false;
	bool slv0_fifo_en  = false;
	mpu_word_t words = 0; /* sensors written to fifo at each sampling time */

	
	if (val & TEMP_FIFO_EN_BIT)	{ temp_fifo_en  = true; words += 1; }
	if (val & XG_FIFO_EN_BIT)	{ xg_fifo_en    = true; words += 1; }
	if (val & YG_FIFO_EN_BIT)	{ yg_fifo_en    = true; words += 1; }
	if (val & ZG_FIFO_EN_BIT)	{ zg_fifo_en    = true; words += 1; }
	if (val & ACCEL_FIFO_EN_BIT)	{ accel_fifo_en = true; words += 3; }
	if (val & SLV2_FIFO_EN_BIT)	{ slv2_fifo_en  = true; words += 1; }
	if (val & SLV1_FIFO_EN_BIT)	{ slv1_fifo_en  = true; words += 1; }
	if (val & SLV0_FIFO_EN_BIT)	{ slv0_fifo_en  = true; words += 1; }

	dev->cfg->temp_fifo_en  = temp_fifo_en;
	dev->cfg->xg_fifo_en    = xg_fifo_en;
	dev->cfg->yg_fifo_en    = yg_fifo_en;
	dev->cfg->zg_fifo_en    = zg_fifo_en;
	dev->cfg->accel_fifo_en = accel_fifo_en;
	dev->cfg->slv2_fifo_en  = slv2_fifo_en;
	dev->cfg->slv1_fifo_en  = slv1_fifo_en;
	dev->cfg->slv0_fifo_en  = slv0_fifo_en;

	/* CRUCIAL - raw[0] = buffered sensors  */
	dev->dat->raw[0] = words;

	return 0;
}

static int mpu_cfg_parse_ACCEL_CONFIG(struct mpu_dev * dev)
{
	if ((NULL == dev) || (NULL == dev->cfg)) 
		return -1;
	
	mpu_reg_t val;
	if ((mpu_cfg_get_val(dev, ACCEL_CONFIG, &val)) < 0)
		return -1; 

	dev->cfg->xa_st = (val & XA_ST_BIT) ? true : false;
	dev->cfg->ya_st = (val & YA_ST_BIT) ? true : false; 
	dev->cfg->za_st = (val & ZA_ST_BIT) ? true : false; 
	
	int afs_sel = (val & AFS_SEL_BIT) >> 3;
	switch (afs_sel) {
		case 0 : dev->afr =  2; dev->albs = 16384; break;
		case 1 : dev->afr =  4; dev->albs =  8192; break;
		case 2 : dev->afr =  8; dev->albs =  4096; break;
		case 3 : dev->afr = 16; dev->albs =  2048; break;
		default	:
			return -1; /* invalid value */
	}
	
	return 0;
}

static int mpu_cfg_parse_GYRO_CONFIG(struct mpu_dev * dev)
{
	
	if ((NULL == dev) || (NULL == dev->cfg)) 
		return -1;
	
	mpu_reg_t val;
	if ((mpu_cfg_get_val(dev, GYRO_CONFIG, &val)) < 0)
		return -1; 

	dev->cfg->xg_st = (val & XG_ST_BIT) ? true : false;
	dev->cfg->yg_st = (val & YG_ST_BIT) ? true : false; 
	dev->cfg->zg_st = (val & ZG_ST_BIT) ? true : false; 

	int fs_sel = (val & FS_SEL_BIT) >> 3;
	switch (fs_sel) {
		case 0 : dev->gfr =  250; dev->glbs = 131.0; break;
		case 1 : dev->gfr =  500; dev->glbs =  65.5; break;
		case 2 : dev->gfr = 1000; dev->glbs =  32.8; break;
		case 3 : dev->gfr = 2000; dev->glbs =  16.4; break;
		default	:
	
			return -1; /* invalid value */
	}

	
	return 0;
}

static int mpu_cfg_parse_CONFIG(struct mpu_dev * dev)
{
	
	if ((NULL == dev) || (NULL == dev->cfg)) 
		return -1;
	
	mpu_reg_t val;
	if ((mpu_cfg_get_val(dev, CONFIG, &val)) < 0)
		return -1; 

	/* ignore EXT_SYNC_SET to avoid complicating senors readings */
	if (val & 0x38) { /* EXT_SYNC_SET not supported */
	       return -1;	
	}

	int dlpf_cfg = (uint8_t)(val & DLPF_CFG_BIT);
	switch (dlpf_cfg) {
		case 0 : dev->abdw = 260; dev->adly =  0;
		         dev->gbdw = 256; dev->gdly =  0.98; 
		         dev->gor  = 8000; 
		         break;
		case 1 : dev->abdw = 184; dev->adly =  2.0; 
		         dev->gbdw = 188; dev->gdly =  1.90; 
		         dev->gor  = 1000; 
		         break;
		case 2 : dev->abdw =  94; dev->adly =  3.0;
		         dev->gbdw =  98; dev->gdly =  2.80;
		         dev->gor  =  1000;
		         break;
		case 3 : dev->abdw =  44; dev->adly =  4.9;
		         dev->gbdw =  42; dev->gdly =  4.8;
		         dev->gor  =  1000;
		         break;
		case 4 : dev->abdw =  21; dev->adly =  8.5;
		         dev->gbdw =  20; dev->gdly =  8.3;
		         dev->gor  =  1000;
		         break;
		case 5 : dev->abdw =  10; dev->adly = 13.8;
		         dev->gbdw =  10; dev->gdly = 13.4;
		         dev->gor  =  1000;
		         break;
		case 6 : dev->abdw =   5; dev->adly = 19.0;
		         dev->gbdw =   5; dev->gdly = 18.6;
		         dev->gor  = 1000;
		         break;
		case 7 : 
	
			return -1; /* error: reserved value */
		default :
	
			return -1; /* error: invalid value */
	}

	
	return 0;
}


static int mpu_cfg_parse_SMPLRT_DIV(struct mpu_dev * dev)
{
	
	if ((NULL == dev) || (NULL == dev->cfg)) 
		return -1;
	
	
	if (dev->gor == 0) /* must have gyro output rate */
		return -1;	

	
	mpu_reg_t val;
	if ((mpu_cfg_get_val(dev, SMPLRT_DIV, &val)) < 0)
		return -1; 

	
	 /* all guaranteed to be greater than zero */
	double gyro_output_rate	  = (double)dev->gor; 
	double samplerate_divisor = (double)(val + 1);
	double sampling_rate	  = gyro_output_rate / samplerate_divisor;
	double sampling_time	  = 1 / sampling_rate;

	dev->sr   = sampling_rate; 
	dev->st	  = sampling_time;

	
	return 0;
}

static int mpu_cfg_parse_INT_PIN_CFG(struct mpu_dev * dev)
{
	if ((NULL == dev) || (NULL == dev->cfg)) 
		return -1;
	
	mpu_reg_t val;
	if ((mpu_cfg_get_val(dev, INT_PIN_CFG, &val)) < 0)
		return -1; 

	if (val & FSYNC_INT_EN_BIT) /* FSYNC_INT_EN not supported */
		return -1;

	if (val & FSYNC_INT_EN_BIT) { dev->cfg->fsync_int_en = true; }

	return 0;
}

static int mpu_cfg_parse_INT_ENABLE(struct mpu_dev * dev)
{
	if ((NULL == dev) || (NULL == dev->cfg)) 
		return -1;
	
	mpu_reg_t val;
	if ((mpu_cfg_get_val(dev, INT_ENABLE, &val)) < 0)
		return -1; 

	bool i2c_mst_en	   = false;
	bool fifo_oflow_en = false;
	bool data_rdy_en   = false;
	if (val & FIFO_OFLOW_EN_BIT)	{ fifo_oflow_en = true; }
	if (val & I2C_MST_INT_EN_BIT)	{ i2c_mst_en = true;	}
	if (val & DATA_RDY_EN_BIT)	{ data_rdy_en = true;	}

	if (i2c_mst_en)	/* I2C_MST_EN not supported */
		return -1;

	dev->cfg->fifo_oflow_en = fifo_oflow_en ;
	dev->cfg->i2c_mst_en	= i2c_mst_en;
	dev->cfg->data_rdy_en   = data_rdy_en;

	return 0;
}

int mpu_dat_set(struct mpu_dev * dev)
{
	if ((NULL == dev) || (NULL == dev->dat) || (NULL == dev->cal)) 
		return -1;

	/* Associate data with meaningful names */
	int count = 1;
	if (dev->cfg->accel_fifo_en) {
		dev->dat->scl[count] = 1.0L/(double)dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->Ax  = &dev->dat->dat[count][0];
		dev->Ax2 = &dev->dat->squ[count];
		dev->Axo = &dev->cal->off[count];
		dev->Axg = &dev->cal->gai[count];
		dev->Axd = &dev->cal->dri[count];
		dev->Axm = &dev->dat->mea[count];
		dev->Axv = &dev->dat->var[count];
		count++;
		dev->dat->scl[count] = 1.0L/(double)dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->Ay  = &dev->dat->dat[count][0];
		dev->Ay2 = &dev->dat->squ[count];
		dev->Ayo = &dev->cal->off[count];
		dev->Ayg = &dev->cal->gai[count];
		dev->Ayd = &dev->cal->dri[count];
		dev->Aym = &dev->dat->mea[count];
		dev->Ayv = &dev->dat->var[count];
		count++;
		dev->dat->scl[count] = 1.0L/(double)dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->Az  = &dev->dat->dat[count][0];
		dev->Az2 = &dev->dat->squ[count];
		dev->Azo = &dev->cal->off[count];
		dev->Azg = &dev->cal->gai[count];
		dev->Azd = &dev->cal->dri[count];
		dev->Azm = &dev->dat->mea[count];
		dev->Azv = &dev->dat->var[count];
		count++;
	}
	if (dev->cfg->temp_fifo_en)	{
		dev->dat->scl[count] = 1/340.0L;
		dev->dat->dat[count][0] = 0;
		dev->t  = &dev->dat->dat[count][0];
		dev->to = &dev->cal->off[count];
		dev->tg = &dev->cal->gai[count];
		dev->td = &dev->cal->dri[count];
		dev->tm = &dev->dat->mea[count];
		dev->tv = &dev->dat->var[count];
		count++;
	}
	if (dev->cfg->xg_fifo_en)	{
		dev->dat->scl[count] = 1.0L/(double)dev->glbs;
		dev->dat->dat[count][0] = 0;
		dev->Gx  = &dev->dat->dat[count][0];
		dev->Gx2 = &dev->dat->squ[count];
		dev->Gxo = &dev->cal->off[count];
		dev->Gxg = &dev->cal->gai[count];
		dev->Gxd = &dev->cal->dri[count];
		dev->Gxm = &dev->dat->mea[count];
		dev->Gxv = &dev->dat->var[count];
		count++;
	}
	if (dev->cfg->yg_fifo_en)	{
		dev->dat->scl[count] = 1.0L/(double)dev->glbs;
		dev->dat->dat[count][0] = 0;
		dev->Gy  = &dev->dat->dat[count][0];
		dev->Gy2 = &dev->dat->squ[count];
		dev->Gyo = &dev->cal->off[count];
		dev->Gyg = &dev->cal->gai[count];
		dev->Gyd = &dev->cal->dri[count];
		dev->Gym = &dev->dat->mea[count];
		dev->Gyv = &dev->dat->var[count];
		count++;
	}
	if (dev->cfg->zg_fifo_en)	{
		dev->dat->scl[count] = 1.0L/(double)dev->glbs;
		dev->dat->dat[count][0] = 0;
		dev->Gz  = &dev->dat->dat[count][0];
		dev->Gz2 = &dev->dat->squ[count];
		dev->Gzo = &dev->cal->off[count];
		dev->Gzg = &dev->cal->gai[count];
		dev->Gzd = &dev->cal->dri[count];
		dev->Gzm = &dev->dat->mea[count];
		dev->Gzv = &dev->dat->var[count];
		count++;
	}
	/* don't know if the ordering is correct */
	if (dev->cfg->slv0_fifo_en) {
		dev->dat->scl[count] = 1/dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->slv0_dat = &dev->dat->dat[count][0];
		count++;
	}
	if (dev->cfg->slv1_fifo_en) {
		dev->dat->scl[count] = 1/dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->slv1_dat = &dev->dat->dat[count][0];
		count++;
	}
	if (dev->cfg->slv2_fifo_en) {
		dev->dat->scl[count] = 1/dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->slv2_dat = &dev->dat->dat[count][0];
		count++; }
	if (dev->cfg->slv3_fifo_en) {
		dev->dat->scl[count] = 1/dev->albs;
		dev->slv3_dat = &dev->dat->dat[count][0];
		count++;
	}
	if (dev->cfg->slv4_fifo_en) {
		dev->dat->scl[count] = 1/dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->slv4_dat = &dev->dat->dat[count][0];
		count++;
	}

	return 0;
}


int mpu_dat_reset(struct mpu_dev * dev)
{
	if (NULL == dev) 
		return -1;

	int i;
	size_t len = (sizeof(dev->dat->scl)/sizeof(mpu_data_t));

	for(i = 1; i < len; i++) {
		dev->dat->scl[i] = 1;
		dev->dat->dat[i][0] = 0;
	}

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
	dev->slv0_dat = NULL;
	dev->slv1_dat = NULL;
	dev->slv2_dat = NULL;
	dev->slv3_dat = NULL;

	return 0;
}

static int mpu_cfg_parse(struct mpu_dev * dev)
{
	if ((NULL == dev) || (NULL == dev->cfg)) 
		return -1;

	if ((mpu_cfg_parse_PWR_MGMT(dev)) < 0 )
	         return -1;
	if ((mpu_cfg_parse_FIFO_EN(dev)) < 0 )
	         return -1;
	if ((mpu_cfg_parse_ACCEL_CONFIG(dev)) < 0 )
	         return -1;
	if ((mpu_cfg_parse_GYRO_CONFIG(dev)) < 0 )
	         return -1;
	if ((mpu_cfg_parse_CONFIG(dev)) < 0 )
	         return -1;
	if ((mpu_cfg_parse_SMPLRT_DIV(dev)) < 0)
		return -1;
	if ((mpu_cfg_parse_USER_CTRL(dev)) < 0)
		return -1;
	if ((mpu_cfg_parse_INT_PIN_CFG(dev)) < 0 )
	         return -1;
	if ((mpu_cfg_parse_INT_ENABLE(dev)) < 0 )
	         return -1;

	return 0;
}

int mpu_cal_reset(struct mpu_dev * dev)
{
	if ((NULL == dev) || (NULL == dev->cal))
		return -1;

	int i = 0;
	size_t len = (sizeof(dev->cal->off)/sizeof(mpu_data_t));
	for (i = 0; i < len; i++) {
		dev->cal->gai[i] = 1;
		dev->cal->off[i] = 0;
		dev->cal->dri[i] = 0;
	}
	dev->cal->gra =1;

	return 0;
}

static int mpu_dev_allocate(struct mpu_dev **dev)
{
	
	if (*dev != NULL) /* dev not empty */
		return -1;
	
	if ((*dev = (struct mpu_dev *)calloc(1, sizeof(struct mpu_dev))) == NULL)
		goto exit_dev;
	
	if (((*dev)->bus = (int *)calloc(1, sizeof(int))) == NULL)
		goto exit_dev_bus;
	
	if (((*dev)->cfg = (struct mpu_cfg *)calloc(1, sizeof(struct mpu_cfg))) == NULL)
		goto exit_dev_cfg;
	
	if (((*dev)->cal = (struct mpu_cal *)calloc(1, sizeof(struct mpu_cal))) == NULL)
		goto exit_dev_cal;
	
	if (((*dev)->dat = (struct mpu_dat *)calloc(1, sizeof(struct mpu_dat))) == NULL)
		goto exit_dev_dat;

	
	return 0;

	/* ensure pointers are NULL after free() */
exit_dev_dat:	free((*dev)->dat); (*dev)->dat = NULL;
exit_dev_cal:	free((*dev)->cal); (*dev)->cal = NULL;
exit_dev_cfg:	free((*dev)->cfg); (*dev)->cfg = NULL;
exit_dev_bus:	free((*dev)->bus); (*dev)->bus = NULL;
exit_dev:	free(*dev); dev = NULL;
	
	return -1;
}

static int mpu_dev_bind(const char * path, const mpu_reg_t address, struct mpu_dev * dev)
{
	
	if ((NULL == dev) || (NULL == dev->bus)) /* invalid pointers */
		return -1;
	
	if ((address == (mpu_reg_t)0x00)) /* invalid address */
		return -1;
	
	size_t pathlen = strlen(path);
	if ((path == NULL) || (pathlen < 6)) /* invalid path */
		return -1;

	
	int fd;
	if ((fd = open(path, O_RDWR)) < 0) /* open failed */
		return -1;	

	/* From now on, fails must close the file descritor */
	if ((ioctl(fd, I2C_SLAVE, address)) < 0) /* bus error */
		goto dev_bind_exit;

	/* success */
	*(dev->bus) = fd;
	dev->addr = address;
	
	return 0;	

dev_bind_exit: 
	if ((close(fd)) < 0) /* cleanup or exit */
		exit(EXIT_FAILURE);

	return -1;
}

int mpu_diagnose(struct mpu_dev * dev)
{
	if(MPUDEV_IS_NULL(dev))
		return -1;

	printf("%-20s %6d\n"	  ,"File Descriptor"	, *(dev->bus));
	printf("%-20s %#6x\n"	  ,"ADDRESS"		, dev->addr);
	printf("%-20s %#6x\n"	  ,"PRODUCT ID"		, dev->prod_id);
	printf("%-20s %6.0lf %s\n","Wake frequency"	, dev->wake_freq	,"(Hz)");
	printf("%-20s %6.0lf %s\n","CLOCK frequency"	, dev->clock_freq	,"(Hz)");
	printf("%-20s %6d %s\n"	  ,"Gyro Output Rate"	, dev->gor		,"(Hz)");
	printf("%-20s %6.0lf %s\n","Sampling rate"	, dev->sr		,"(Hz)");
	printf("%-20s %6.0lf %s\n","Sampling time"	, dev->st		,"(s)");
	printf("%-20s %6.0lf %s\n","Accel Full Range"	, dev->afr		,"(g)");
	printf("%-20s %6.0lf %s\n","Accel LBS"		, dev->albs		,"(LSB/g)");
	printf("%-20s %6.0lf %s\n","Accel bandwidth"	, dev->abdw		,"(Hz)");
	printf("%-20s %6.0lf %s\n","Accel delay"	, dev->adly		,"(ms)");
	printf("%-20s %6.0lf %s\n","Gyro Full Range"	, dev->gfr		,"(degrees/s)");
	printf("%-20s %6.0lf %s\n","Gyro LBS"		, dev->glbs		,"(LSB/degress/s)");
	printf("%-20s %6.0lf %s\n","Gyro bandwidth"	, dev->gbdw		,"(Hz)");
	printf("%-20s %6.0lf %s\n","Gyro delay"		, dev->gdly		,"(ms)");
	printf("----------------------------------------\n");

	printf("%-20s %d\n","SLEEP BIT"		, dev->cfg->sleep );
	printf("%-20s %d\n","CYCLE BIT"		, dev->cfg->cycle );
	printf("%-20s %d\n","TEMP_DIS BIT"	, dev->cfg->temp_dis );
	printf("%-20s %d\n","STDBY_XA BIT"	, dev->cfg->stdby_xa );
	printf("%-20s %d\n","STDBY_YA BIT"	, dev->cfg->stdby_ya );
	printf("%-20s %d\n","STDBY_ZA BIT"	, dev->cfg->stdby_za );
	printf("%-20s %d\n","STDBY_XG BIT"	, dev->cfg->stdby_xg );
	printf("%-20s %d\n","STDBY_YG BIT"	, dev->cfg->stdby_yg );
	printf("%-20s %d\n","STDBY_ZG BIT"	, dev->cfg->stdby_zg );
	printf("%-20s %d\n","I2C_MST_EN BIT"	, dev->cfg->i2c_mst_en );
	printf("%-20s %d\n","I2C_IF_DIS BIT"	, dev->cfg->i2c_if_dis );
	printf("%-20s %d\n","FIFO_EN BIT"	, dev->cfg->fifo_en );
	printf("%-20s %d\n","TEMP_FIFO_EN BIT"	, dev->cfg->temp_fifo_en );
	printf("%-20s %d\n","FIFO Accel BIT"	, dev->cfg->accel_fifo_en );
	printf("%-20s %d\n","FIFO Gyro X BIT"	, dev->cfg->xg_fifo_en );
	printf("%-20s %d\n","FIFO Gyro Y BIT"	, dev->cfg->yg_fifo_en );
	printf("%-20s %d\n","FIFO Gyro Z BIT"	, dev->cfg->zg_fifo_en );
	printf("%-20s %d\n","FIFO SLV0"		, dev->cfg->slv0_fifo_en );
	printf("%-20s %d\n","FIFO SLV1"		, dev->cfg->slv1_fifo_en );
	printf("%-20s %d\n","FIFO SLV2"		, dev->cfg->slv2_fifo_en );
	printf("%-20s %d\n","FIFO SLV3"		, dev->cfg->slv3_fifo_en );
	printf("%-20s %d\n","FIFO SLV4"		, dev->cfg->slv4_fifo_en );
	printf("%-20s %d\n","FSYNC_INT_EN BIT"	, dev->cfg->fsync_int_en );
	printf("%-20s %d\n","FIFO_OFLOW_EN BIT"	, dev->cfg->fifo_oflow_en );
	printf("%-20s %d\n","I2C_MST_INT_EN BIT", dev->cfg->i2c_mst_int_en );
	printf("%-20s %d\n","DATA_RDY_EN BIT"	, dev->cfg->data_rdy_en );
	printf("%-20s %d\n","raw[0]"		, dev->dat->raw[0] );
	printf("----------------------------------------\n");

	printf("ADDRESSES:\n");
	printf("dat[1](%p) Ax=%p\n", dev->dat->dat[1], dev->Ax);
	printf("dat[2](%p) Ay=%p\n", dev->dat->dat[2], dev->Ay);
	printf("dat[3](%p) Az=%p\n", dev->dat->dat[3], dev->Az);
	printf("dat[4](%p) t =%p\n", dev->dat->dat[4], dev->t);
	printf("dat[5](%p) Gx=%p\n", dev->dat->dat[5], dev->Gx);
	printf("dat[6](%p) Gy=%p\n", dev->dat->dat[6], dev->Gy);
	printf("dat[7](%p) Gz=%p\n", dev->dat->dat[7], dev->Gz);
	printf("----------------------------------------\n");
	printf("SCALING:\n");
	printf("scl[1](%lf) %lf\n", dev->dat->scl[1], 1/dev->albs);
	printf("scl[2](%lf) %lf\n", dev->dat->scl[2], 1/dev->albs);
	printf("scl[3](%lf) %lf\n", dev->dat->scl[3], 1/dev->albs);
	printf("scl[4](%lf) %lf\n", dev->dat->scl[4], 1/340.0);
	printf("scl[5](%lf) %lf\n", dev->dat->scl[5], 1/dev->glbs);
	printf("scl[6](%lf) %lf\n", dev->dat->scl[6], 1/dev->glbs);
	printf("scl[7](%lf) %lf\n", dev->dat->scl[7], 1/dev->glbs);
	printf("----------------------------------------\n");

	return 0;
}

int mpu_ctl_fifo_data(struct mpu_dev *dev)
{
	if (MPUDEV_IS_NULL(dev))
		return -1;

	int len = 1 + dev->dat->raw[0];
	if (1 == len) {
		return 0;
	}


	int count = 0;
	if ((mpu_ctl_fifo_count(dev, &count)) < 0)
		return -1;

	if (count > 1023) { /* buffer overflow */
		mpu_ctl_fifo_flush(dev);
	} else {
		while (count < len) { /* buffer underflow */
			mpu_ctl_fifo_count(dev, &count);
		}
	}

	for (int i = 1; i < len; i++) {
		if (mpu_fifo_data(dev, &dev->dat->raw[i]) < 0)
			return -1;
		dev->dat->dat[i][0] = dev->dat->raw[i] * dev->dat->scl[i];
		dev->dat->dat[i][1] = dev->dat->dat[i][0];
		dev->dat->squ[i]    = dev->dat->dat[i][0] * dev->dat->dat[i][0];
	}
	if (dev->cfg->temp_fifo_en) {
		*(dev->t) += 36.53;
	}

	return 0;
}

int mpu_ctl_fifo_count(struct mpu_dev *dev, int *count)
{
	if (MPUDEV_IS_NULL(dev))
		return -1;

	uint16_t tem = 0;
	if ((mpu_read_word(dev, FIFO_COUNT_H, &tem)) < 0)
		return -1;

	uint16_t buf = (tem << 8) | (tem >> 8);
	*count = buf;

	return 0;
}

int mpu_ctl_fifo_flush(struct mpu_dev *dev)
{
	if (MPUDEV_IS_NULL(dev))
		return -1;

	uint8_t dat = 0;
	int count = 0;
       	mpu_ctl_fifo_count(dev, &count);
	for (int i = 0; i < count; i++) {
		if ((mpu_read_byte(dev, FIFO_R_W, &dat)) < 0)
			return -1;
	}

	return 0;
}

static int mpu_fifo_data(struct mpu_dev *dev, int16_t *data)
{

	/* HIC SUNT DRACONES */

	uint16_t dh;  /* unsigned for bit fiddling data high */
	uint16_t dl;  /* unsigned for bit fiddling data low  */
	uint8_t buf;
	if (mpu_read_byte(dev, FIFO_R_W, &buf) < 0)
		return -1;
	dh = (uint16_t)(buf & 0x00FF) << 8;

	if (mpu_read_byte(dev, FIFO_R_W, &buf) < 0)
		return -1;
	dl = (uint16_t)(buf & 0x00FF);
	
	*data = (uint16_t)(dh | dl);
	return 0;
}

int mpu_read_data(struct mpu_dev * const dev, const mpu_reg_t reg, int16_t * val)
{
	uint16_t dh;  /* unsigned for bit fiddling data high */
	uint16_t dl;  /* unsigned for bit fiddling data low  */
	uint8_t buf;
	if (mpu_read_byte(dev, reg, &buf) < 0)
		return -1;
	dh = (uint16_t)(buf & 0x00FF) << 8;

	if (mpu_read_byte(dev, reg+1, &buf) < 0)
		return -1;
	dl = (uint16_t)(buf & 0x00FF);
	
	*val = (uint16_t)(dh | dl);
	return 0;
}

int mpu_ctl_selftest(struct mpu_dev * dev)
{
	/* prepare the device for self-test */
	struct mpu_cfg *cfg_old = calloc(1, sizeof(struct mpu_cfg));
	memcpy((void *)cfg_old,  (void *)dev->cfg, sizeof(struct mpu_cfg));

	mpu_ctl_dlpf(dev,0);
	mpu_ctl_samplerate(dev, 100);
	mpu_ctl_clocksource(dev, 3);
	mpu_ctl_accel_range(dev, 8);
	mpu_ctl_gyro_range(dev, 250);
	mpu_ctl_fifo_disable_accel(dev);
	mpu_ctl_fifo_disable_temp(dev);
	mpu_ctl_fifo_disable_gyro(dev);

	long double xa_st_on  = 0;
	long double ya_st_on  = 0;
	long double za_st_on  = 0;
	long double xg_st_on  = 0;
	long double yg_st_on  = 0;
	long double zg_st_on  = 0;
	long double xa_st_off = 0;
	long double ya_st_off = 0;
	long double za_st_off = 0;
	long double xg_st_off = 0;
	long double yg_st_off = 0;
	long double zg_st_off = 0;

	int samples = 100;
	/* Accel self-test response */
	mpu_ctl_fifo_enable_accel(dev);
	mpu_ctl_selftest_enable_accel(dev);
	for (int i = 0; i < samples; i++) {
		mpu_ctl_fifo_data(dev);
		xa_st_on += *(dev->Ax);
		ya_st_on += *(dev->Ay);
		za_st_on += *(dev->Az);
	}
	mpu_ctl_selftest_disable_accel(dev);
	for (int i = 0; i < samples; i++) {
		mpu_ctl_fifo_data(dev);
		xa_st_off += *(dev->Ax);
		ya_st_off += *(dev->Ay);
		za_st_off += *(dev->Az);
	}
	mpu_ctl_fifo_disable_accel(dev);
	/* Gyro self-test response */
	mpu_ctl_fifo_enable_gyro(dev);
	mpu_ctl_selftest_enable_gyro(dev);
	for (int i = 0; i < samples; i++) {
		mpu_ctl_fifo_data(dev);
		xg_st_on += *(dev->Gx);
		yg_st_on += *(dev->Gy);
		zg_st_on += *(dev->Gz);
	}
	mpu_ctl_selftest_disable_gyro(dev);
	for (int i = 0; i < samples; i++) {
		mpu_ctl_fifo_data(dev);
		xg_st_off += *(dev->Gx);
		yg_st_off += *(dev->Gy);
		zg_st_off += *(dev->Gz);
	}
	mpu_ctl_fifo_disable_gyro(dev);

	long double xa_str = fabsl(xa_st_on - xa_st_off)/(long double)samples;
	long double ya_str = fabsl(ya_st_on - ya_st_off)/(long double)samples;
	long double za_str = fabsl(za_st_on - za_st_off)/(long double)samples;
	long double xg_str = fabsl(xg_st_on - xg_st_off)/(long double)samples;
	long double yg_str = fabsl(yg_st_on - yg_st_off)/(long double)samples;
	long double zg_str = fabsl(zg_st_on - zg_st_off)/(long double)samples;

	/* get self-test registers */
	uint8_t stx = 0;
	uint8_t sty = 0;
	uint8_t stz = 0;
	uint8_t sta = 0;
	mpu_read_byte(dev, SELF_TEST_X, &stx);
	mpu_read_byte(dev, SELF_TEST_Y, &sty);
	mpu_read_byte(dev, SELF_TEST_Z, &stz);
	mpu_read_byte(dev, SELF_TEST_A, &sta);
	uint8_t xa_st = ((stx & XA_TEST_42_BIT) >> 3) | ((sta & XA_TEST_10_BIT) >> 4);
	uint8_t ya_st = ((sty & YA_TEST_42_BIT) >> 3) | ((sta & YA_TEST_10_BIT) >> 2);
	uint8_t za_st = ((stz & ZA_TEST_42_BIT) >> 3) |  (sta & ZA_TEST_10_BIT);
	uint8_t xg_st =  (stx & XG_TEST_40_BIT);
	uint8_t yg_st =  (sty & YG_TEST_40_BIT);
	uint8_t zg_st =  (stz & ZG_TEST_40_BIT);

	/* ACCEL get factory trim values - datasheet formula for +-8g */
	long double ft_xa = 4096.0L * 0.34L * powl(0.92L/0.34L, ((long double)(xa_st -1)/14.0L));
	long double ft_ya = 4096.0L * 0.34L * powl(0.92L/0.34L, ((long double)(ya_st -1)/14.0L));
	long double ft_za = 4096.0L * 0.34L * powl(0.92L/0.34L, ((long double)(za_st -1)/14.0L));
	/* GYRO get factory trim values - datahsset fomula for +-250 dps */
	long double ft_xg =  25.0L * 131.0L * powl(1046L, (long double)(xg_st -1));
	long double ft_yg = -25.0L * 131.0L * powl(1046L, (long double)(yg_st -1));
	long double ft_zg =  25.0L * 131.0L * powl(1046L, (long double)(zg_st -1));

	long double shift_xa = (xa_str - ft_xa)/ft_xa;
	long double shift_ya = (ya_str - ft_ya)/ft_ya;
	long double shift_za = (za_str - ft_za)/ft_za;
	long double shift_xg = (xg_str - ft_xg)/ft_xg;
	long double shift_yg = (yg_str - ft_yg)/ft_yg;
	long double shift_zg = (zg_str - ft_zg)/ft_zg;
	printf("Self-test results: Xa = %Lf%% shift from factory trim (%4s)\n", fabsl(shift_xa), fabsl(shift_xa) < 14.0L ? "PASS" : "FAIL");
	printf("Self-test resutls: Ya = %Lf%% shift from factory trim (%4s)\n", fabsl(shift_ya), fabsl(shift_ya) < 14.0L ? "PASS" : "FAIL");
	printf("Self-test resutls: Za = %Lf%% shift from factory trim (%4s)\n", fabsl(shift_za), fabsl(shift_za) < 14.0L ? "PASS" : "FAIL");
	printf("Self-test resutls: Xg = %Lf%% shift from factory trim (%4s)\n", fabsl(shift_xg), fabsl(shift_xg) < 14.0L ? "PASS" : "FAIL");
	printf("Self-test resutls: Yg = %Lf%% shift from factory trim (%4s)\n", fabsl(shift_yg), fabsl(shift_yg) < 14.0L ? "PASS" : "FAIL");
	printf("Self-test resutls: Zg = %Lf%% shift from factory trim (%4s)\n", fabsl(shift_zg), fabsl(shift_zg) < 14.0L ? "PASS" : "FAIL");

	/* restore old config */
	memcpy((void *)dev->cfg, (void *)cfg_old, sizeof(struct mpu_cfg));
	mpu_cfg_set(dev);
	mpu_dat_set(dev);
	free(cfg_old);
	cfg_old = NULL;

	return 0;
}


int mpu_ctl_fifo_reset(struct mpu_dev *dev)
{
	uint8_t val;
	if (mpu_cfg_get_val(dev, USER_CTRL, &val) < 0)
		return -1;

	if (mpu_write_byte(dev, USER_CTRL, val | FIFO_RESET_BIT ) < 0)
		return -1;

	sleep(1);
	if (mpu_ctl_fifo_flush(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_i2c_mst_reset(struct mpu_dev *dev)
{
	uint8_t val;
	if (mpu_cfg_get_val(dev, USER_CTRL, &val) < 0)
		return -1;

	if (mpu_write_byte(dev, USER_CTRL, val | I2C_MST_RESET_BIT ) < 0)
		return -1;

	sleep(1);
	if (mpu_ctl_fifo_flush(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_sig_cond_reset(struct mpu_dev *dev)
{
	uint8_t val;
	if (mpu_cfg_get_val(dev, USER_CTRL, &val) < 0)
		return -1;

	if (mpu_write_byte(dev, USER_CTRL, val | SIG_COND_RESET_BIT ) < 0)
		return -1;

	sleep(1);
	if (mpu_ctl_fifo_flush(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_fifo_enable(struct mpu_dev *dev)
{
	uint8_t val = 0;
	if (mpu_cfg_get_val(dev, USER_CTRL, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, USER_CTRL, val | FIFO_EN_BIT ) < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;


	return 0;
}

int mpu_ctl_fifo_enable_accel(struct mpu_dev *dev)
{
	uint8_t val;
	if (mpu_cfg_get_val(dev, FIFO_EN, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, FIFO_EN, val | ACCEL_FIFO_EN_BIT)  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_fifo_enable_gyro(struct mpu_dev *dev)
{
	uint8_t val;
	if (mpu_cfg_get_val(dev, FIFO_EN, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, FIFO_EN, val | XG_FIFO_EN_BIT | YG_FIFO_EN_BIT | ZG_FIFO_EN_BIT)  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_fifo_enable_temp(struct mpu_dev *dev)
{
	uint8_t val;
	if (mpu_cfg_get_val(dev, FIFO_EN, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, FIFO_EN, val | TEMP_FIFO_EN_BIT)  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;
	
	return 0;
}

int mpu_ctl_fifo_disable(struct mpu_dev *dev)
{
	uint8_t val = 0;
	if (mpu_cfg_get_val(dev, USER_CTRL, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, USER_CTRL, val & (~FIFO_EN_BIT) ) < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;
	sleep(1);

	return 0;
}

int mpu_ctl_fifo_disable_accel(struct mpu_dev *dev)
{
	uint8_t val;
	if (mpu_cfg_get_val(dev, FIFO_EN, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, FIFO_EN, val & (~ACCEL_FIFO_EN_BIT))  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_fifo_disable_gyro(struct mpu_dev *dev)
{
	uint8_t val;
	if (mpu_cfg_get_val(dev, FIFO_EN, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, FIFO_EN, val & (~(XG_FIFO_EN_BIT | YG_FIFO_EN_BIT | ZG_FIFO_EN_BIT)))  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_fifo_disable_temp(struct mpu_dev *dev)
{
	uint8_t val;
	if (mpu_cfg_get_val(dev, FIFO_EN, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, FIFO_EN, val & (~TEMP_FIFO_EN_BIT))  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_selftest_fifo_disable(struct mpu_dev *dev)
{
	uint8_t val = 0;
	if (mpu_cfg_get_val(dev, USER_CTRL, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, USER_CTRL, val & (~FIFO_EN_BIT) ) < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;
	sleep(1);

	val = ~(ACCEL_FIFO_EN_BIT | XG_FIFO_EN_BIT | YG_FIFO_EN_BIT | ZG_FIFO_EN_BIT);
	if (mpu_cfg_set_val(dev, FIFO_EN, val)  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_selftest_enable_accel(struct mpu_dev *dev)
{
	uint8_t val;
	if (mpu_cfg_get_val(dev, ACCEL_CONFIG, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, ACCEL_CONFIG, val | XA_ST_BIT | YA_ST_BIT | ZA_ST_BIT)  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	sleep(1);
	if (mpu_ctl_fifo_flush(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_selftest_enable_gyro(struct mpu_dev *dev)
{
	uint8_t val;
	if (mpu_cfg_get_val(dev, GYRO_CONFIG, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, GYRO_CONFIG, val | XG_ST_BIT | YG_ST_BIT | ZG_ST_BIT)  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	sleep(1);
	if (mpu_ctl_fifo_flush(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_selftest_disable_accel(struct mpu_dev *dev)
{
	uint8_t val;
	if (mpu_cfg_get_val(dev, ACCEL_CONFIG, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, ACCEL_CONFIG, val & (~(XA_ST_BIT | YA_ST_BIT | ZA_ST_BIT)))  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	sleep(1);
	if (mpu_ctl_fifo_flush(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_selftest_disable_gyro(struct mpu_dev *dev)
{
	uint8_t val;
	if (mpu_cfg_get_val(dev, GYRO_CONFIG, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, GYRO_CONFIG, val & (~(XG_ST_BIT | YG_ST_BIT | ZG_ST_BIT)))  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	sleep(1);
	if (mpu_ctl_fifo_flush(dev) < 0)
		return -1;

	return 0;
}

void mpu_print_data(struct mpu_dev *dev, int times)
{
	for (int i = 0; i < times; i++) {
		printf("%3d Hz:%#+3.0lf abdw=%#+4.2lf gbdw=%#+4.2lf  afr=%#+4.2lf  xfr=%#+4.2lf ", i, dev->sr, dev->abdw, dev->gbdw, dev->afr, dev->gfr);
		for (int j = 1; j < (1 + dev->dat->raw[0]); j++) {
			printf(" D[%d]=%+3.4lf", j, dev->dat->dat[j][0]);
		}
		printf("\n");
	}
}

void mpu_print_all(struct mpu_dev *dev, int times)
{
	for (int i = 0; i < times; i++) {
		printf("%3d Hz:%#+3.0lf abdw=%#+4.2lf gbdw=%#+4.2lf  afr=%#+4.2lf  xfr=%#+4.2lf ",
			       i, dev->sr, dev->abdw, dev->gbdw, dev->afr, dev->gfr);
		printf("Hz:%3.0lf Ax:%+lf Ay:%+lf Az:%+lf Gx:%+lf Gy:%+lf Gz:%+lf Temperature:%+lf\n",
				dev->sr,
				*(dev->Ax), *(dev->Ay), *(dev->Az),
				*(dev->Gx), *(dev->Gy), *(dev->Gz),
				*(dev->t));
	}
}
//int mpu_ctl_selftest_driver(struct mpu_dev * dev)
//{
//	/* prepare the device for self-test */
//	struct mpu_cfg *cfg_old = calloc(1, sizeof(struct mpu_cfg));
//	memcpy((void *)cfg_old,  (void *)dev->cfg, sizeof(struct mpu_cfg));
//	/* evaluate self-test response */
//	long double (* st_off)[16] = calloc(16, sizeof(long double));
//	long double  (* st_on)[16] = calloc(16, sizeof(long double));
//	mpu_ctl_selftest_get_data(dev, st_off, false);
//	mpu_ctl_selftest_get_data(dev, st_on , true);
//	/* get self-test registers */
//	uint8_t stx = 0;
//	uint8_t sty = 0;
//	uint8_t stz = 0;
//	uint8_t sta = 0;
//	mpu_read_byte(dev, SELF_TEST_X, &stx);
//	mpu_read_byte(dev, SELF_TEST_Y, &sty);
//	mpu_read_byte(dev, SELF_TEST_Z, &stz);
//	mpu_read_byte(dev, SELF_TEST_A, &sta);
//	uint8_t xa_st = ((stx & XA_TEST_42_BIT) >> 3) | ((sta & XA_TEST_10_BIT) >> 4);
//	uint8_t ya_st = ((sty & YA_TEST_42_BIT) >> 3) | ((sta & YA_TEST_10_BIT) >> 2);
//	uint8_t za_st = ((stz & ZA_TEST_42_BIT) >> 3) |  (sta & ZA_TEST_10_BIT);
//	uint8_t xg_st =  (stx & XG_TEST_40_BIT);
//	uint8_t yg_st =  (sty & YG_TEST_40_BIT);
//	uint8_t zg_st =  (stz & ZG_TEST_40_BIT);
//	printf(" xa_st = %Lf\n", (long double)xa_st); 
//	printf(" ya_st = %Lf\n", (long double)ya_st); 
//	printf(" za_st = %Lf\n", (long double)za_st); 
//	printf(" xg_st = %Lf\n", (long double)xg_st); 
//	printf(" yg_st = %Lf\n", (long double)yg_st); 
//	printf(" zg_st = %Lf\n", (long double)zg_st); 
//	/* ACCEL get factory trim values - formula inside inv_mpu.c, get_accel_prod_shift */
//	long double ft_xa = 0.34L * (powl(0.92L/0.34L, ((long double)(xa_st -1)/14.0L))/30.0L);
//	long double ft_ya = 0.34L * (powl(0.92L/0.34L, ((long double)(ya_st -1)/14.0L))/30.0L);
//	long double ft_za = 0.34L * (powl(0.92L/0.34L, ((long double)(za_st -1)/14.0L))/30.0L);
//	/* ACCEL evaluate reponse */
//	long double xa_cus  = fabsl((* st_off)[0] - (long double)xa_st)/65536.0L;
//	long double ya_cus  = fabsl((* st_off)[1] - (long double)ya_st)/65536.0L;
//	long double za_cus  = fabsl((* st_off)[2] - (long double)za_st)/65536.0L;
//	long double xa_var  = xa_cus/(ft_xa - 1.0L);
//	long double ya_var  = ya_cus/(ft_ya - 1.0L);
//	long double za_var  = za_cus/(ft_za - 1.0L);
//	/* ACCEL evaluate reponse */
//	long double xa_cus  = fabsl((* st_off)[0] - (long double)xa_st)/65536.0L;
//	long double ya_cus  = fabsl((* st_off)[1] - (long double)ya_st)/65536.0L;
//	long double za_cus  = fabsl((* st_off)[2] - (long double)za_st)/65536.0L;
//	long double xa_var  = xa_cus/(ft_xa - 1.0L);
//	long double ya_var  = ya_cus/(ft_ya - 1.0L);
//	long double za_var  = za_cus/(ft_za - 1.0L);
//	printf(" ft_xa  = %Lf\n", ft_xa ); 
//	printf(" ft_ya  = %Lf\n", ft_ya ); 
//	printf(" ft_za  = %Lf\n", ft_za ); 
//	printf(" xa_cus = %Lf\n", xa_cus); 
//	printf(" ya_cus = %Lf\n", ya_cus); 
//	printf(" za_cus = %Lf\n", za_cus); 
//	printf(" xa_var = %Lf\n", xa_var); 
//	printf(" ya_var = %Lf\n", ya_var); 
//	printf(" za_var = %Lf\n", za_var); 
//	/* ACCEL evaluate tests */
//	long double max_accel_var = 0.14L;
//	long double max_g = 0.95L;
//	long double min_g = 0.3L;
//	long double max_xa_bias = (0.23L * 65535.0L);
//	long double max_ya_bias = (0.23L * 65535.0L);
//	long double max_za_bias = (0.18L * 65535.0L);
//	printf("%-10s %Lf %Lf %s\n", "xa_var",		fabsl(xa_var),	max_accel_var,	fabsl(xa_var) > max_accel_var	? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "ya_var",		fabsl(ya_var),	max_accel_var,	fabsl(ya_var) > max_accel_var	? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "za_var",		fabsl(za_var),	max_accel_var,	fabsl(za_var) > max_accel_var	? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "xa_cus",		xa_cus,		max_g,		xa_cus > max_g			? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "ya_cus",		ya_cus,		max_g,		ya_cus > max_g 			? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "za_cus",		za_cus,		max_g,		za_cus > max_g 			? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "xa_cus",		xa_cus,		min_g,		xa_cus < min_g 			? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "ya_cus",		ya_cus,		min_g,		ya_cus < min_g 			? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "za_cus",		za_cus,		min_g,		za_cus < min_g 			? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "st_off[3]",	(* st_off)[0],	max_xa_bias,	(* st_off)[0] > max_xa_bias	? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "st_off[4]",	(* st_off)[1],	max_ya_bias,	(* st_off)[1] > max_ya_bias 	? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "st_off[5]",	(* st_off)[2],	max_za_bias,	(* st_off)[2] > max_za_bias 	? "fail": "pass");
//	/* GYRO get factory trim values - formula nside inv_mpu.c, get_accel_prod_shift */
//	long double ft_xg = (3275.0L/dev->glbs) * powl(1046L, (long double)(xg_st -1));
//	long double ft_yg = (3275.0L/dev->glbs) * powl(1046L, (long double)(yg_st -1));
//	long double ft_zg = (3275.0L/dev->glbs) * powl(1046L, (long double)(zg_st -1));
//	/* GYRO evaluate reponse */
//	long double xg_cus  = fabsl((* st_off)[3] - (long double)xg_st)/65536.0L;
//	long double yg_cus  = fabsl((* st_off)[4] - (long double)yg_st)/65536.0L;
//	long double zg_cus  = fabsl((* st_off)[5] - (long double)zg_st)/65536.0L;
//	long double xg_var  = xg_cus/(ft_xa - 1.0L);
//	long double yg_var  = yg_cus/(ft_ya - 1.0L);
//	long double zg_var  = zg_cus/(ft_za - 1.0L);
//	/* GYRO evaluate tests */
//	long double max_gyro_var = 0.14L;
//	long double min_dps = 10.0L;
//	long double max_dps = 105.0L;
//	long double max_gyro_bias = (20.0L * 65535.0);
//	printf("%-10s %Lf %Lf %s\n", "xg_var",		fabsl(xg_var),	max_gyro_var,	fabsl(xg_var) > max_gyro_var	? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "yg_var",		fabsl(yg_var),	max_gyro_var,	fabsl(yg_var) > max_gyro_var	? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "zg_var",		fabsl(zg_var),	max_gyro_var,	fabsl(zg_var) > max_gyro_var	? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "xg_cus",		xg_cus,		max_dps,	xg_cus > max_dps		? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "yg_cus",		yg_cus,		max_dps,	yg_cus > max_dps		? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "zg_cus",		zg_cus,		max_dps,	zg_cus > max_dps		? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "xg_cus",		xg_cus,		min_dps,	xg_cus < min_dps		? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "yg_cus",		yg_cus,		min_dps,	yg_cus < min_dps		? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "zg_cus",		zg_cus,		min_dps,	zg_cus < min_dps		? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "st_off[3]",	(* st_off)[3],	max_gyro_bias,	(* st_off)[3] > max_gyro_bias	? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "st_off[4]",	(* st_off)[4],	max_gyro_bias,	(* st_off)[4] > max_gyro_bias	? "fail": "pass");
//	printf("%-10s %Lf %Lf %s\n", "st_off[5]",	(* st_off)[5],	max_gyro_bias,	(* st_off)[5] > max_gyro_bias	? "fail": "pass");
//
//	/* restore old config */
//	memcpy((void *)dev->cfg, (void *)cfg_old, sizeof(struct mpu_cfg));
//	mpu_cfg_set(dev);
//	mpu_dat_set(dev);
//	free(cfg_old);
//	cfg_old = NULL;
//	return 0;
//}
//
//int mpu_ctl_selftest_get_data(struct mpu_dev *dev, long double (* buffer)[16], bool tri)
//{
//	memcpy((void *)dev->cfg, (void *)&mpu6050_stcfg, sizeof(struct mpu_cfg));
//	mpu_cfg_set(dev);
//	mpu_dat_reset(dev);
//	mpu_dat_set(dev);
//	mpu_ctl_fifo_disable(dev);
//	mpu_ctl_fifo_flush(dev);
//	mpu_ctl_fifo_reset(dev);
//	mpu_ctl_fifo_enable(dev);
//	if (tri) { mpu_ctl_selftest_enable_accel(dev); }
//
//	/* Save the average value */
//	mpu_ctl_fifo_enable(dev);
//	int len = 1 + dev->dat->raw[0];
//	int samples = 50;
//	mpu_ctl_fifo_flush(dev);
//	for (int i = 0; i < samples; i++) {
//		mpu_ctl_fifo_data(dev);
//		for (int j = 1; j < len; j++) {
//			(*buffer)[j] += dev->dat->dat[j][0];
//		}
//	 }
//	printf("Averaged over %d samples\n", samples);
//	for (int j = 1; j < len; j++) {
//		(*buffer)[j] /= (long double)samples;
//		printf("buffer[%d]=%Lf\n", j, (*buffer)[j]);
//	}
//
//	mpu_ctl_fifo_disable(dev);
//	mpu_ctl_fifo_flush(dev);
//	mpu_ctl_fifo_reset(dev);
//	return 0;
//
//}
