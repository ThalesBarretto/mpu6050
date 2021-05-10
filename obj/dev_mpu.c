struct mpu_dev {
	/* internal data - managed through special handlers */
	int		bus;		/* bus file decriptor			*/
	uint8_t		addr;		/* device i2c bus address		*/
	struct mpu_cfg	*cfg;		/* config register state		*/
	struct mpu_dat	*dat;		/* sensor readings			*/
	struct mpu_cal	*cal;		/* calibration data			*/
	struct mpu_pdat	*pdat;		/* processed data			*/
	struct mpu_regs	*regs;		/* register dump			*/
	/* readable config - result of special handlers */
	uint8_t		aen, gen, ten;	/* sensor enabled state			*/
	uint16_t	bufsize;	/* fifo buffer size			*/
	uint16_t	gor;		/* gyro output rate			*/
	double	sr, st, sdiv;		/* sampling rate, time, rate divisor	*/
	double	afr, albs, abdw, adly;	/* range, lbs, bandwidth, delay		*/
	double	gfr, glbs, gbdw, gdly;
	unsigned long long samples;	/* sample counter			*/
	/* readable data */
	double	*Ax, *Axo, *Axg, *Axdr;	/* reading, offset, gain, drift rate	*/
	double	*Ay, *Ayo, *Ayg, *Aydr;
	double	*Az, *Azo, *Azg, *Azdr;
	double	*Gx, *Gxo, *Gxg, *Gxdr;
	double	*Gy, *Gyo, *Gyg, *Gydr;
	double	*Gz, *Gzo, *Gzg, *Gzdr;
	double	*t, *to, *tg, *dr;
}

struct mpu_cal {
	double	gravity;		/* mean(sqrt(ax2,ay2,az2)[])		*/
	double	offsets[16];		/* ax - mean(ax[n]), ...		*/
	double	gains[16];		/* ax[expec_1g]/ax[measured_1g], ...	*/
	double	driftrate[16];		/* delta(mean(ax[n])/delta(time), ...	*/
	double	ypr_accel[4];
}

struct mpu_dat {
	int16_t	raw[16];	/* raw sensor data	*/
	double	scl[16];	/* scaling factors	*/
	double	dat[16][2];	/* scaled data		*/
}

struct mpu_cfg {
	uint8_t	cfg[10][2];	/* configuration state	*/
}

struct mpu_pdat {
	double	squares[16];

}

struct mpu_regs {
	uint8_t	dump[100][3];
}
