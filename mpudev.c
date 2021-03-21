
/*
 * struct mpu_device
 *
 * Maps configuration bits of the device.
 * Each member is erlated to one configuration bi set.
 * The actual translation to the device registers is handled separetely.
 */
struct mpu_device {

	int clksel
	int dlpf_cfg
	int smplrt_div

//	int ext_sync_set

	int fs_sel
	int afs_sel

}

/*
 * MPU DEVICE FUNCTIONS
 *
 */

	/*
	 * data sampling control
	 *
	 * Clock Source selection
	 * Digital Lowpass filter mode selection
	 * Samplerate divisor selection
	 * External Frame Synchronizaton selection
	 * TODO: Handle valid sampling rate values
	 *
	 * CLKSEL Clock Source
	 * 0 Internal 8MHz oscillator
	 * 1 PLL with X axis gyroscope reference
	 * 2 PLL with Y axis gyroscope reference
	 * 3 PLL with Z axis gyroscope reference
	 * 4 PLL with external 32.768kHz reference
	 * 5 PLL with external 19.2MHz reference
	 * 6 RESERVED
	 * 7 Stops the clock snf keeps the timing generator in reset
	 *
	 * DLPF_CFG	FS
	 * 0		8 kHz
	 * 1		1 kHz
	 * 2		1 kHz
	 * 3		1 kHz
	 * 4		1 kHz
	 * 5		1 kHz
	 * 6		1 kHz
	 * 7		RESERVED
	 *
	 * SMPLRT_DIV 8-bit unsigned value.
	 *
	 * THE SAMPLE RATE
	 * ---------------
	 *
	 * Sensor register output, FIFO output and DMP sampling are based on Sample Rate.
	 * Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7)
	 * Gyroscope Output Rate = 1kHz when the DLPF is enabled (DLPF_CFG = 1-6)
	 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	 *
	 * gyro_output_rate = (dlpf == 0) || (dlpf == 7) ? 8000 : 1000;
	 * smplrt_div = (Gyro_output_rate/Sample_rate) -1;
	 *
	 * Sensible Sample Rate options ( >2*bandwidth):
	 * DLPF_CFG	Samplerate
	 * 0		>512 Hz
	 * 1		>376 Hz
	 * 2		>196 Hz
	 * 3		>84 Hz
	 * 4		>40 Hz
	 * 5		>20 Hz
	 * 6		>10 Hz
	 * 7		RESERVED
	 */

	void mpu_set_clksel(struct mpu_device *dev);
	void mpu_set_dlpf(struct mpu_device *dev);
	void mpu_set_smplrt_div(struct mpu_device *dev);
	/*
	 * data range control
	 * Select full scale range of sensors
	 * TODO: Handle valid full range values
	 * FS_SEL	RANGE
	 * 0		+-250 deg/s
	 * 1		+-500 deg/s
	 * 2		+-1000 deg/s
	 * 3		+-2000 deg/s
	 * AFS_SEL	RANGE
	 * 0		+-2g
	 * 1		+-4g
	 * 2		+-8g
	 * 3		+-16g
	 */


void mpu_fifo_count(struct mpu_device *dev);
void mpu_fifo_en(struct mpu_device *dev);
void mpu_accel_fifo_en(struct mpu_device *dev);
void mpu_xg_fifo_en(struct mpu_device *dev);
void mpu_yg_fifo_en(struct mpu_device *dev);
void mpu_zg_fifo_en(struct mpu_device *dev);
void mpu_temp_fifo_en(struct mpu_device *dev);

/*
 * self-tests
 * Setting bit triggers self-test on sensor.
 * Results are stored in specific registers.
 * Refer to the datasheet for making sense of the results
 * TODO: Implement PASS/FAIL routine
 */
void mpu_xa_st(struct mpu_device *dev);
void mpu_ya_st(struct mpu_device *dev);
void mpu_za_st(struct mpu_device *dev);
void mpu_xg_st(struct mpu_device *dev);
void mpu_yg_st(struct mpu_device *dev);
void mpu_zg_st(struct mpu_device *dev);
int xa_test
int ya_test
int za_test
int xg_test
int yg_test
int zg_test

/*
 * interrupt control
 * Configure the behaviour of the generation of interrupt signals on INT pin
 * TODO: Read Interrupt status, handle values
 */
void mpu_int_level(struct mpu_device *dev);
void mpu_int_open(struct mpu_device *dev);
void mpu_latch_int_en(struct mpu_device *dev);
void mpu_int_rd_clear(struct mpu_device *dev);
void mpu_fsync_int_level(struct mpu_device *dev);
void mpu_fifo_oflow_en(struct mpu_device *dev);
void mpu_data_rdy_en(struct mpu_device *dev);
void mpu_fsync_int_en(struct mpu_device *dev);
void mpu_i2c_bypass_en(struct mpu_device *dev);
void mpu_i2c_mst_int_en(struct mpu_device *dev);
/* Check interrupts */
void mpu_fifo_oflow_int
void mpu_i2c_mst_int
void mpu_data_rdy_int


/* SECONDARY BUS
I2C_MST_EN
When set to 1, this bit enables I C Master Mode.
When this bit is cleared to 0, the auxiliary I C bus lines (AUX_DA and AUX_CL) are
logically driven by the primary I C bus (SDA and SCL).
I2C_IF_DIS
MPU-6000: When set to 1, this bit disables the primary I C interface and enables the SPI interface instead.
MPU-6050: Always write this bit as zero.
*/
int i2c_mst_en
int i2c_if_dis
/* data fifo - secondary */
int i2c_mst_en
int i2c_if_dis
int slv0_fifo_en
int slv1_fifo_en
int slv2_fifo__en
int slv3__fifo_en
	int mult_mst_en
int i2c_mst_p_nsr
int i2c_mst_clk
int wait_for_es
/*...*/

/* bus data flow */
int delay_es_shadow
int i2c_slv4_dly_en
int i2c_slv3_dly_en
int i2c_slv2_dly_en
int i2c_slv1_dly_en
int i2c_slv0_dly_en

/*
 * reset controls
 *
 * Setting the bit triggers reset function.
 * The device will take some time to recover from a reset.
 */
void mpu_deviceice_reset(struct mpu_device *dev);
void mpu_sig_cond_reset(struct mpu_device *dev);
void mpu_accel_reset(struct mpu_device *dev);
void mpu_gyro_reset(struct mpu_device *dev);
void mpu_temp_reset(struct mpu_device *dev);
void mpu_fifo_reset(struct mpu_device *dev);
void mpu_i2c_mst_reset(struct mpu_device *dev);

/*
 * power controls
 *
 * Setting the bit toggles power mode
 * Sleeping device doesn't accept commands.
 * Before disabling gyros check if they are clock sources.
 * If needed, changed to internal oscillator before disabling or sleeping.
 *
 * TODO: Store this infromation in the deivce struct
 */
void mpu_sleep(struct mpu_device *dev);
void mpu_cycle(struct mpu_device *dev);
void mpu_lp_wake_ctrl(struct mpu_device *dev);
void mpu_stdby_xa(struct mpu_device *dev);
void mpu_stdby_ya(struct mpu_device *dev);
void mpu_stdby_za(struct mpu_device *dev);
void mpu_stdby_xg(struct mpu_device *dev);
void mpu_stdby_yg(struct mpu_device *dev);
void mpu_stdby_zg(struct mpu_device *dev);
void mpu_temp_dis(struct mpu_device *dev);

/*
 * who am i (read-only)
 *
 * stores the device bus address.
 * TODO: Store this infromation in the deivce struct
 */
void mpu_who_am_i(struct mpu_device *dev);

