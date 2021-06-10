
/*
 * MPU6050 device description
 * Include this in your .c file
 */

/*
 * Orientation of Sensor Axis
 *
 * Axis should be individually checked for: sequence, polarity, matching.
 *  I has been fount that:s
 * 	Accelerometer readings are inverse-polarity with respect to sensor body
 *	Gyroscope is correct with respect to the sensor body
 *
 * The correct orientation and polarity is as shown:
 *  (compare with: Product specification rev. 3.4., p. 40, 46.)
 */
#define MSG_AXIS "" \
"---------------------------------------------------\n" \
"      MPU-6050 FACTORY SENSOR AXIS SETUP           \n" \
"---------------------------------------------------\n" \
"                      (-Az,+Gz CCW)                \n" \
"                          ^                        \n" \
"                          | (-Ay,+Gy CCW)          \n" \
"       sensor body        |   ^		    \n" \
"        (top view)        |  /                     \n" \
"       ____________       | /                      \n" \
"      /0          /'      |/                       \n" \
"     / MPU-60050 /.       0---------->(-Ax,+Gx CCW)\n" \
"    /           /.        Standard Axis            \n" \
"   /           /. 	   Configuration            \n" \
"  /___________/.   				    \n" \
"  '...........'    				    \n" \
"                                                   \n" \
"___________________________________________________\n"


/*
 * MPU-6050 register definitions
 *
 * Here are defined all registers documented in:
 * MPU-6000/6050 Product Specification rev. 3.4. (08/19/2013) PS-MPU-6000A-00
 * MPU-6000/6050 Register Map and Desription rev. 4.12. (08/19/2013) RM-MPU-6000A-00
 * MPU Hardware Offset Registers Application Note rev 1.0 (02/21/2014) AN-XX-XXXX-XX
 *
 * Accel offset registers
 * 1. Initial values contain factory trim values. Read value, apply bias;
 * 2. Format is in +-8g scale (1mg = 4096 LSB)
 * 3. Bit 0 on the low byte of each axis is reserved and must be unchanged.
 *
 * Gyro offset registers
 * 1. Initial values are 0;
 * 2. Format is in +-1000dps scale (1dps = 32.8 LSB)
 * 3. Bit 0 on the low byte of each axis is reserved and must be unchanged.
 */
//0 0x00
#define AUX_VDDIO		  1
//2 0x02
//3 0x03
//4 0x04
//5 0x05
#define XA_OFFS_USRH		  6 // 0x06
#define XA_OFFS_USRL		  7 // 0x07
#define YA_OFFS_USRH		  8 // 0x08
#define YA_OFFS_USRL		  9 // 0x09
#define ZA_OFFS_USRH		 10 // 0x0A
#define ZA_OFFS_USRL		 11 // 0x0B
#define PROD_ID			 12 // 0X0C // new
#define SELF_TEST_X		 13
#define SELF_TEST_Y		 14
#define SELF_TEST_Z		 15
#define SELF_TEST_A		 16
//17 0x11
//18 0x12
#define XG_OFFS_USRH		 19 //0x13
#define XG_OFFS_USRL		 20 //0x14
#define YG_OFFS_USRH		 21 //0x15
#define YG_OFFS_USRL		 22 //0x16
#define ZG_OFFS_USRH		 23 //0x17
#define ZG_OFFS_USRL		 24 //0x18
#define SMPLRT_DIV		 25
#define CONFIG			 26
#define GYRO_CONFIG		 27
#define ACCEL_CONFIG		 28
#define FF_THR			 29
#define FF_DUR			 30
#define MOT_THR			 31
#define MOT_DUR			 32
#define ZRMOT_THR		 33
#define ZRMOT_DUR		 34
#define FIFO_EN			 35
#define I2C_MST_CTRL		 36
#define I2C_SLV0_ADDR		 37
#define I2C_SLV0_REG		 38
#define I2C_SLV0_CTRL		 39
#define I2C_SLV1_ADDR		 40
#define I2C_SLV1_REG		 41
#define I2C_SLV1_CTRL		 42
#define I2C_SLV2_ADDR		 43
#define I2C_SLV2_REG		 44
#define I2C_SLV2_CTRL		 45
#define I2C_SLV3_ADDR		 46
#define I2C_SLV3_REG		 47
#define I2C_SLV3_CTRL		 48
#define I2C_SLV4_ADDR		 49
#define I2C_SLV4_REG		 50
#define I2C_SLV4_DO		 51
#define I2C_SLV4_CTRL		 52
#define I2C_SLV4_DI		 53
#define I2C_MAST_STATUS		 54
#define INT_PIN_CFG		 55
#define INT_ENABLE		 56
#define DMP_INT_STATUS		 57 // new
#define INT_STATUS		 58
#define ACCEL_XOUT_H		 59
#define ACCEL_XOUT_L		 60
#define ACCEL_YOUT_H		 61
#define ACCEL_YOUT_L		 62
#define ACCEL_ZOUT_H 		 63
#define ACCEL_ZOUT_L		 64
#define TEMP_OUT_H		 65
#define TEMP_OUT_L		 66
#define GYRO_XOUT_H		 67
#define GYRO_XOUT_L		 68
#define GYRO_YOUT_H		 69
#define GYRO_YOUT_L		 70
#define GYRO_ZOUT_H		 71
#define GYRO_ZOUT_L		 72
#define EXT_SENS_DATA_00	 73
#define EXT_SENS_DATA_01	 74
#define EXT_SENS_DATA_02	 75
#define EXT_SENS_DATA_03	 76
#define EXT_SENS_DATA_04	 77
#define EXT_SENS_DATA_05	 78
#define EXT_SENS_DATA_06	 79
#define EXT_SENS_DATA_07	 70
#define EXT_SENS_DATA_08	 81
#define EXT_SENS_DATA_09	 82
#define EXT_SENS_DATA_10	 83
#define EXT_SENS_DATA_11	 84
#define EXT_SENS_DATA_12	 85
#define EXT_SENS_DATA_13	 86
#define EXT_SENS_DATA_14	 87
#define EXT_SENS_DATA_15	 88
#define EXT_SENS_DATA_16	 89
#define EXT_SENS_DATA_17	 90
#define EXT_SENS_DATA_18	 91
#define EXT_SENS_DATA_19	 92
#define EXT_SENS_DATA_20	 93
#define EXT_SENS_DATA_21	 94
#define EXT_SENS_DATA_22	 95
#define EXT_SENS_DATA_23	 96
#define MOT_DETECT_STATUS	 97
//98 0x62
#define I2C_SLV0_DO		 99
#define I2C_SLV1_DO		100
#define I2C_SLV2_DO		101
#define I2C_SLV3_DO		102
#define I2C_MST_DELAY_CTRL	103
#define SIGNAL_PATH_RESET	104
#define MOT_DETECT_CTRL		105
#define USER_CTRL		106
#define PWR_MGMT_1		107
#define PWR_MGMT_2		108
#define BANK_SEL 		109 // new
#define MEM_START_ADDR 		110 // new
#define MEM_R_W			111 // new
#define PRGM_START_H		112 // new
#define PRGM_START_L		113 // new
#define FIFO_COUNT_H		114
#define FIFO_COUNT_L		115
#define FIFO_R_W 		116
#define WHO_AM_I		117

/* MPU6050 bit range definitions */
#define XA_TEST_42_BIT		(uint8_t)(0xE0u) /* SELF_TEST_X */
#define XG_TEST_40_BIT		(uint8_t)(0x1Fu) /* SELF_TEST_X */
#define YA_TEST_42_BIT		(uint8_t)(0xE0u) /* SELF_TEST_Y */
#define YG_TEST_40_BIT		(uint8_t)(0x1Fu) /* SELF_TEST_Y */
#define ZA_TEST_42_BIT		(uint8_t)(0xE0u) /* SELF_TEST_Z */
#define ZG_TEST_40_BIT		(uint8_t)(0x1Fu) /* SELF_TEST_Z */
#define XA_TEST_10_BIT		(uint8_t)(0x30u) /* SELF_TEST_A */
#define YA_TEST_10_BIT		(uint8_t)(0x0Cu) /* SELF_TEST_A */
#define ZA_TEST_10_BIT		(uint8_t)(0x03u) /* SELF_TEST_A */

#define XG_ST_BIT		(uint8_t)(0x80u) /* GYRO_CONFIG */
#define YG_ST_BIT		(uint8_t)(0x40u) /* GYRO_CONFIG */
#define ZG_ST_BIT		(uint8_t)(0x20u) /* GYRO_CONFIG */
#define XA_ST_BIT		(uint8_t)(0x80u) /* ACCEL_CONFIG */
#define YA_ST_BIT		(uint8_t)(0x40u) /* ACCEL_CONFIG */
#define ZA_ST_BIT		(uint8_t)(0x20u) /* ACCEL_CONFIG */

#define FIFO_EN_BIT		(uint8_t)(0x40u) /* USER_CTRL */
#define I2C_MST_EN_BIT		(uint8_t)(0x20u) /* USER_CTRL */
#define I2C_IF_DIS_BIT		(uint8_t)(0x10u) /* USER_CTRL */
#define FIFO_RESET_BIT		(uint8_t)(0x04u) /* USER CTRL */
#define I2C_MST_RESET_BIT	(uint8_t)(0x02u) /* USER CTRL */
#define SIG_COND_RESET_BIT	(uint8_t)(0x01u) /* USER CTRL */

#define GYRO_RESET_BIT		(uint8_t)(0x04u) /* SIGNAL_PATH_RESET */
#define ACCEL_RESET_BIT		(uint8_t)(0x02u) /* SIGNAL_PATH_RESET */
#define TEMP_RESET_BIT		(uint8_t)(0x01u) /* SIGNAL_PATH_RESET */

#define DEVICE_RESET_BIT	(uint8_t)(0x80u) /* PWR_MGMT_1 */
#define SLEEP_BIT		(uint8_t)(0x40u) /* PWR_MGMT_1 */
#define CYCLE_BIT		(uint8_t)(0x20u) /* PWR_MGMT_1 */
#define TEMP_DIS_BIT		(uint8_t)(0x08u) /* PWR_MGMT_1 */
#define CLKSEL_BIT		(uint8_t)(0x07u) /* PWR_MGMT_1 */
#define CLKSEL_0		(uint8_t)(0x00u) /* PWR_MGMT_1 */
#define CLKSEL_1		(uint8_t)(0x01u) /* PWR_MGMT_1 */
#define CLKSEL_2		(uint8_t)(0x02u) /* PWR_MGMT_1 */
#define CLKSEL_3		(uint8_t)(0x03u) /* PWR_MGMT_1 */
#define CLKSEL_4		(uint8_t)(0x04u) /* PWR_MGMT_1 */
#define CLKSEL_5		(uint8_t)(0x05u) /* PWR_MGMT_1 */
#define CLKSEL_6		(uint8_t)(0x06u) /* PWR_MGMT_1 */
#define CLKSEL_7		(uint8_t)(0x07u) /* PWR_MGMT_1 */

#define LP_WAKE_CTL_BIT		(uint8_t)(0xC0u) /* PWR_MGMT_2 */
#define STDBY_XA_BIT		(uint8_t)(0x20u) /* PWR_MGMT_2 */
#define STDBY_YA_BIT		(uint8_t)(0x10u) /* PWR_MGMT_2 */
#define STDBY_ZA_BIT		(uint8_t)(0x08u) /* PWR_MGMT_2 */
#define STDBY_XG_BIT		(uint8_t)(0x04u) /* PWR_MGMT_2 */
#define STDBY_YG_BIT		(uint8_t)(0x02u) /* PWR_MGMT_2 */
#define STDBY_ZG_BIT		(uint8_t)(0x01u) /* PWR_MGMT_2 */
#define LP_WAKE_CTL_0		(uint8_t)(0x00u) /* PWR_MGMT_2 */
#define LP_WAKE_CTL_1		(uint8_t)(0x40u) /* PWR_MGMT_2 */
#define LP_WAKE_CTL_2		(uint8_t)(0x80u) /* PWR_MGMT_2 */
#define LP_WAKE_CTL_3		(uint8_t)(0xC0u) /* PWR_MGMT_2 */

#define DLPF_CFG_BIT		(uint8_t)(0x07u) /* CONFIG */
#define DLPF_CFG_0		(uint8_t)(0x00u) /* CONFIG */
#define DLPF_CFG_1		(uint8_t)(0x01u) /* CONFIG */
#define DLPF_CFG_2		(uint8_t)(0x02u) /* CONFIG */
#define DLPF_CFG_3		(uint8_t)(0x03u) /* CONFIG */
#define DLPF_CFG_4		(uint8_t)(0x04u) /* CONFIG */
#define DLPF_CFG_5		(uint8_t)(0x05u) /* CONFIG */
#define DLPF_CFG_6		(uint8_t)(0x06u) /* CONFIG */
#define DLPF_CFG_7		(uint8_t)(0x07u) /* CONFIG */

#define FS_SEL_BIT		(uint8_t)(0x18u) /* GYRO_CONFIG */
#define FS_SEL_0		(uint8_t)(0x00u) /* GYRO_CONFIG */
#define FS_SEL_1		(uint8_t)(0x08u) /* GYRO_CONFIG */
#define FS_SEL_2		(uint8_t)(0x10u) /* GYRO_CONFIG */
#define FS_SEL_3		(uint8_t)(0x18u) /* GYRO_CONFIG */

#define AFS_SEL_BIT		(uint8_t)(0x18u) /* ACCEL_CONFIG */
#define AFS_SEL_0		(uint8_t)(0x00u) /* ACCEL_CONFIG */
#define AFS_SEL_1		(uint8_t)(0x08u) /* ACCEL_CONFIG */
#define AFS_SEL_2		(uint8_t)(0x10u) /* ACCEL_CONFIG */
#define AFS_SEL_3		(uint8_t)(0x18u) /* ACCEL_CONFIG */

#define TEMP_FIFO_EN_BIT	(uint8_t)(0x80u) /* FIFO_EN */
#define XG_FIFO_EN_BIT		(uint8_t)(0x40u) /* FIFO_EN */
#define YG_FIFO_EN_BIT		(uint8_t)(0x20u) /* FIFO_EN */
#define ZG_FIFO_EN_BIT		(uint8_t)(0x10u) /* FIFO_EN */
#define ACCEL_FIFO_EN_BIT	(uint8_t)(0x08u) /* FIFO_EN */
#define SLV2_FIFO_EN_BIT	(uint8_t)(0x04u) /* FIFO_EN */
#define SLV1_FIFO_EN_BIT	(uint8_t)(0x02u) /* FIFO_EN */
#define SLV0_FIFO_EN_BIT	(uint8_t)(0x01u) /* FIFO_EN */

#define INT_LEVEL_BIT		(uint8_t)(0x80u) /* INT_PIN_CFG */
#define INT_OPEN_BIT		(uint8_t)(0x40u) /* INT_PIN_CFG */
#define LATCH_INT_BIT		(uint8_t)(0x20u) /* INT_PIN_CFG */
#define INT_RD_CLEAR_BIT	(uint8_t)(0x10u) /* INT_PIN_CFG */
#define FSYNC_INT_LEVEL_BIT	(uint8_t)(0x08u) /* INT_PIN_CFG */
#define FSYNC_INT_EN_BIT	(uint8_t)(0x04u) /* INT_PIN_CFG */
#define I2C_BYPASS_EN_BIT	(uint8_t)(0x02u) /* INT_PIN_CFG */

#define FIFO_OFLOW_EN_BIT	(uint8_t)(0x10u) /* INT_ENABLE */
#define I2C_MST_INT_EN_BIT	(uint8_t)(0x08u) /* INT_ENABLE */
#define DATA_RDY_EN_BIT		(uint8_t)(0x01u) /* INT_ENABLE */

#define FIFO_OFLOW_INT_BIT	(uint8_t)(0x10u) /* INT_STATUS */
#define I2C_MST_INT_BIT		(uint8_t)(0x08u) /* INT_STATUS */
#define DATA_RDY_INT_BIT	(uint8_t)(0x01u) /* INT_STATUS */

#define MULT_MST_EN_BIT		(uint8_t)(0x80u) /* I2C_MST_CTRL */
#define WAIT_FOR_ES_BIT		(uint8_t)(0x40u) /* I2C_MST_CTRL */
#define SLV3_FIFO_EN_BIT	(uint8_t)(0x20u) /* I2C_MST_CTRL */
#define I2C_MST_P_NSR_BIT	(uint8_t)(0x10u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_BIT		(uint8_t)(0x0Fu) /* I2C_MST_CTRL */
#define I2C_MST_CLK_0		(uint8_t)(0x00u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_1		(uint8_t)(0x01u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_2		(uint8_t)(0x02u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_3		(uint8_t)(0x03u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_4		(uint8_t)(0x04u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_5		(uint8_t)(0x05u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_6		(uint8_t)(0x06u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_7		(uint8_t)(0x07u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_8		(uint8_t)(0x08u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_9		(uint8_t)(0x09u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_10		(uint8_t)(0x0Au) /* I2C_MST_CTRL */
#define I2C_MST_CLK_11		(uint8_t)(0x0Bu) /* I2C_MST_CTRL */
#define I2C_MST_CLK_12		(uint8_t)(0x0Cu) /* I2C_MST_CTRL */
#define I2C_MST_CLK_13		(uint8_t)(0x0Du) /* I2C_MST_CTRL */
#define I2C_MST_CLK_14		(uint8_t)(0x0Eu) /* I2C_MST_CTRL */
#define I2C_MST_CLK_15		(uint8_t)(0x0Fu) /* I2C_MST_CTRL */

uint8_t mpu_regvalues[128];

char mpu_regnames[ 128 ][ 32 ] = {
[ AUX_VDDIO ]		= "AUX_VDDIO",
[ XA_OFFS_USRH ] 	= "XA_OFFS_USRH",
[ XA_OFFS_USRL ] 	= "XA_OFFS_USRL",
[ YA_OFFS_USRH ] 	= "YA_OFFS_USRH",
[ YA_OFFS_USRL ] 	= "YA_OFFS_USRL",
[ ZA_OFFS_USRH ] 	= "ZA_OFFS_USRH",
[ ZA_OFFS_USRL ] 	= "ZA_OFFS_USRL",
[ XG_OFFS_USRH ] 	= "XG_OFFS_USRH",
[ XG_OFFS_USRL ] 	= "XG_OFFS_USRL",
[ YG_OFFS_USRH ] 	= "YG_OFFS_USRH",
[ YG_OFFS_USRL ] 	= "YG_OFFS_USRL",
[ ZG_OFFS_USRH ] 	= "ZG_OFFS_USRH",
[ ZG_OFFS_USRL ]	= "ZG_OFFS_USRL",
[ PROD_ID ] 		= "PROD_ID",
[ SELF_TEST_X ] 	= "SELF_TEST_X",
[ SELF_TEST_Y ]		= "SELF_TEST_Y",
[ SELF_TEST_Z ]		= "SELF_TEST_Z",
[ SELF_TEST_A ]		= "SELF_TEST_A",
[ SMPLRT_DIV ]		= "SMPLRT_DIV",
[ CONFIG ]		= "CONFIG",
[ GYRO_CONFIG ]		= "GYRO_CONFIF",
[ ACCEL_CONFIG ]	= "ACCEL_CONFIG",
[ FF_THR ]		= "FF_THR",
[ FF_DUR ]		= "FF_DUR",
[ MOT_THR ]		= "MOT_THR",
[ MOT_DUR ]		= "MOT_DUR",
[ ZRMOT_THR ]		= "ZRMOT_THR",
[ ZRMOT_DUR ]		= "ZRMOT_DUR",
[ FIFO_EN ]		= "FIFO_EN",
[ I2C_MST_CTRL ]	= "I2C_MST_CTRL",
[ I2C_SLV0_ADDR ] 	= "I2C_SLV0_ADDR",
[ I2C_SLV0_REG ]	= "I2C_SLV0_REG",
[ I2C_SLV0_CTRL ] 	= "I2C_SLV0_CTRL",
[ I2C_SLV1_ADDR ]	= "I2C_SLV1_ADDR",
[ I2C_SLV1_REG ]	= "I2C_SLV1_REG",
[ I2C_SLV1_CTRL ]	= "I2C_SLV1_CTRL",
[ I2C_SLV2_ADDR ]	= "I2C_SLV2_ADDR",
[ I2C_SLV2_REG ]	= "I2C_SLV2_REG",
[ I2C_SLV2_CTRL ]	= "I2C_SLV2_CTRL",
[ I2C_SLV3_ADDR ]	= "I2C_SLV3_ADDR",
[ I2C_SLV3_REG ]	= "I2C_SLV3_REG",
[ I2C_SLV3_CTRL ]	= "I2C_SLV3_CTRL",
[ I2C_SLV4_ADDR ]	= "I2C_SLV4_ADDR",
[ I2C_SLV4_REG ]	= "I2C_SLV4_REG",
[ I2C_SLV4_DO ]		= "I2C_SLV4_DO",
[ I2C_SLV4_CTRL ]	= "I2C_SLV4_CTRL",
[ I2C_SLV4_DI ]		= "I2C_SLV4_DI",
[ I2C_MAST_STATUS ]	= "I2C_MST_STATUS",
[ INT_PIN_CFG ]		= "INT_PIN_CFG",
[ INT_ENABLE ]		= "INT_ENABLE",
[ DMP_INT_STATUS ] 	= "DMP_INT_STATUS",
[ INT_STATUS ]		= "INT_STATUS",
[ ACCEL_XOUT_H ] 	= "ACCEL_XOUT_H",
[ ACCEL_XOUT_L ] 	= "ACCEL_XOUT_L",
[ ACCEL_YOUT_H ] 	= "ACCEL_YOUT_H",
[ ACCEL_YOUT_L ] 	= "ACCEL_YOUT_L",
[ ACCEL_ZOUT_H ] 	= "ACCEL_ZOUT_H ",
[ ACCEL_ZOUT_L ] 	= "ACCEL_ZOUT_L",
[ TEMP_OUT_H ] 		= "TEMP_OUT_H",
[ TEMP_OUT_L ] 		= "TEMP_OUT_L",
[ GYRO_XOUT_H ] 	= "GYRO_XOUT_H",
[ GYRO_XOUT_L ] 	= "GYRO_XOUT_L",
[ GYRO_YOUT_H ] 	= "GYRO_YOUT_H",
[ GYRO_YOUT_L ] 	= "GYRO_YOUT_L",
[ GYRO_ZOUT_H ] 	= "GYRO_ZOUT_H",
[ GYRO_ZOUT_L ] 	= "GYRO_ZOUT_L",
[ EXT_SENS_DATA_00 ]	= "EXT_SENS_DATA_00",
[ EXT_SENS_DATA_01 ]	= "EXT_SENS_DATA_01",
[ EXT_SENS_DATA_02 ]	= "EXT_SENS_DATA_02",
[ EXT_SENS_DATA_03 ]	= "EXT_SENS_DATA_03",
[ EXT_SENS_DATA_04 ]	= "EXT_SENS_DATA_04",
[ EXT_SENS_DATA_05 ]	= "EXT_SENS_DATA_05",
[ EXT_SENS_DATA_06 ]	= "EXT_SENS_DATA_06",
[ EXT_SENS_DATA_07 ]	= "EXT_SENS_DATA_07",
[ EXT_SENS_DATA_08 ]	= "EXT_SENS_DATA_08",
[ EXT_SENS_DATA_09 ]	= "EXT_SENS_DATA_09",
[ EXT_SENS_DATA_10 ]	= "EXT_SENS_DATA_10",
[ EXT_SENS_DATA_11 ]	= "EXT_SENS_DATA_11",
[ EXT_SENS_DATA_12 ]	= "EXT_SENS_DATA_12",
[ EXT_SENS_DATA_13 ]	= "EXT_SENS_DATA_13",
[ EXT_SENS_DATA_14 ]	= "EXT_SENS_DATA_14",
[ EXT_SENS_DATA_15 ]	= "EXT_SENS_DATA_15",
[ EXT_SENS_DATA_16 ]	= "EXT_SENS_DATA_16",
[ EXT_SENS_DATA_17 ]	= "EXT_SENS_DATA_17",
[ EXT_SENS_DATA_18 ]	= "EXT_SENS_DATA_18",
[ EXT_SENS_DATA_19 ]	= "EXT_SENS_DATA_19",
[ EXT_SENS_DATA_20 ]	= "EXT_SENS_DATA_20",
[ EXT_SENS_DATA_21 ]	= "EXT_SENS_DATA_21",
[ EXT_SENS_DATA_22 ]	= "EXT_SENS_DATA_22",
[ EXT_SENS_DATA_23 ]	= "EXT_SENS_DATA_23",
[ MOT_DETECT_STATUS ]	= "MOT_DETECT_STATUS",
[ I2C_SLV0_DO ]		= "I2C_SLV0_DO",
[ I2C_SLV1_DO ]		= "I2C_SLV1_DO",
[ I2C_SLV2_DO ]		= "I2C_SLV2_DO",
[ I2C_SLV3_DO ]		= "I2C_SLV0_DO",
[ I2C_MST_DELAY_CTRL ]	= "I2C_MST_DELAY_CTRL",
[ SIGNAL_PATH_RESET ]	= "SIGNAL_PATH_RESET",
[ MOT_DETECT_CTRL ]	= "MOT_DETECT_CTRL",
[ USER_CTRL ]		= "USER_CTRL",
[ PWR_MGMT_1 ]		= "PWR_MGMT_1",
[ PWR_MGMT_2 ]		= "PWR_MGMT_2",
[ MEM_R_W ] 		= "MEM_R_W",
[ BANK_SEL ] 		= "BANK_SEL",
[ MEM_START_ADDR ] 	= "MEM_START_ADDR",
[ PRGM_START_H ] 	= "PRGM_START_H",
[ PRGM_START_L ] 	= "PRGM_START_L",
[ FIFO_COUNT_H ]	= "FIFO_COUNT_H",
[ FIFO_COUNT_L ]	= "FIFO_COUNT_L",
[ FIFO_R_W ]		= "FIFO_R_W",
[ WHO_AM_I ]		= "WHO_AM_I",
};

/*
 * Enable device tree for i2c-1 inside /boot/config.txt
 * 	[all]
 * 	dtparam=i2c_arm=on
 * 	dtparam=i2c_arm_baudrate=400000
 *
 * Supported features
 * 	Accelerometer		- enable/disable, range setting
 * 	Gyroscope 	  	- enable/disable, range setting
 * 	Temperature sensor	- enable/disable
 * 	Sampling rate control 	- 10,20,25,50,100,200 Hz
 * 	Buffered readings	- ensures regular sampling interval
 * 	DLPF (Digital Low Pass filter) - refer to datasheet
 *
 * Unsupported features
 * 	Self-tests		- lazy
 * 	eDMP (embedded Digital Motion Proccessor) - blob, undocumented
 * 	Low-power modes		- not our use case
 * 	External interrupts	- not our use case
 * 	External clock sources	- not our use case
 * 	Secondary i2c bus	- not our use case
 *
 * Return value for all function calls:
 * 	On success, return 0.
 * 	On failure, return -1;
 */



#ifndef MPU6050_ADDR
#define MPU6050_ADDR 0x68
#endif

typedef uint8_t  mpu_reg_t;
typedef uint16_t mpu_word_t;
typedef double   mpu_data_t;

struct mpu_dev {
	/* basic interface setting */
	int	*bus;		/* bus file decriptor */
	uint8_t	addr;		/* device i2c bus address */
	uint8_t prod_id;	/* product id */
	/* internal data - managed through special handlers */
	struct	mpu_cfg	*cfg;	/* config register state */
	struct	mpu_dat	*dat;	/* sensor readings */
	struct	mpu_cal	*cal;	/* calibration data */
	/* readable config - result of special handlers */
	bool	aolpm;		/* accelerometer-only low power mode */
	double	wake_freq;	/* low-power cycling freq */
	double	clock_freq;	/* determined by CLKSEL */
	size_t	fifomax;	/* fifo buffer capacity in bytes */
	size_t	fifocnt;	/* bytes available in fifo */
	unsigned int gor;	/* gyro output rate (Hz) */
	double str_xa;		/* self-test response */
	double str_ya;		/* self-test response */
	double str_za;		/* self-test response */
	double str_xg;		/* self-test response */
	double str_yg;		/* self-test response */
	double str_zg;		/* self-test response */
	double ft_xa;		/* factory_trim value */
	double ft_ya;		/* factory_trim value */
	double ft_za;		/* factory_trim value */
	double ft_xg;		/* factory_trim value */
	double ft_yg;		/* factory_trim value */
	double ft_zg;		/* factory_trim value */
	double sr;		/* sampling rate */
	double st;		/* sampling time */
	double afr;		/* accelerometer full range in g-force */
	double albs;		/* accelerometer least bit sensitivity */
	double abdw;		/* accelerometer bandwidth in Hertz (Hz)*/
	double adly;		/* accelerometer delay in miliseconds (ms) */
	double gfr;		/* gyroscope full range in degrees per second */
	double glbs;		/* gyroscope least bit sensitivity */
	double gbdw;		/* gyroscope bandwidth in Hertz (Hz) */
	double gdly;		/* gyroscope delay in miliseconds (ms) */
	/* readable data */
	unsigned long long samples;	/* sample counter			*/
	mpu_data_t	*Ax, *Ax2, *Axo, *Axg, *Axm, *Axv, *Axd;
	mpu_data_t	*Ay, *Ay2, *Ayo, *Ayg, *Aym, *Ayv, *Ayd;
	mpu_data_t	*Az, *Az2, *Azo, *Azg, *Azm, *Azv, *Azd;
	mpu_data_t	*Gx, *Gx2, *Gxo, *Gxg, *Gxm, *Gxv, *Gxd;
	mpu_data_t	*Gy, *Gy2, *Gyo, *Gyg, *Gym, *Gyv, *Gyd;
	mpu_data_t	*Gz, *Gz2, *Gzo, *Gzg, *Gzm, *Gzv, *Gzd;
	mpu_data_t	*t, 	    *to,  *tg,  *tm,  *tv,  *td;
	mpu_data_t	*slv0_dat, *slv1_dat, *slv2_dat, *slv3_dat, *slv4_dat;
};

struct mpu_cal {
	mpu_data_t gra;		/* mean(sqrt(ax2,ay2,az2)[])		*/
	mpu_data_t off[32];	/* ax - mean(ax[n]), ...		*/
	mpu_data_t gai[32];	/* ax[expec_1g]/ax[measured_1g], ...	*/
	mpu_data_t dri[32];	/* delta(mean(ax[n])/delta(time), ...	*/
};

struct mpu_dat {

	int16_t raw[32];	/* raw sensor data	*/

	mpu_data_t scl[32];	/* scaling factors	*/
	mpu_data_t dat[32][2];	/* scaled data		*/
	mpu_data_t squ[32];	/* squared data		*/
	mpu_data_t mea[32];	/* data mean		*/
	mpu_data_t var[32];	/* data variance	*/
};

struct mpu_cfg {
	mpu_reg_t cfg[16][2];	/* configuration state	*/
	/* configuration bits   -> REGISTER	 */
	bool sleep; 		/* PWR_MGMGT_1 */
	bool cycle; 		/* PWR_MGMGT_1 */
	bool temp_dis;		/* PWR_MGMGT_1 */
	bool xg_st;		/* GYRO_CONFIG */
	bool yg_st;		/* GYRO_CONFIG */
	bool zg_st;		/* GYRO_CONFIG */
	bool xa_st;		/* ACCEL_CONFIG */
	bool ya_st;		/* ACCEL_CONFIG */
	bool za_st;		/* ACCEL_CONFIG */
	bool stdby_xa; 		/* PWR_MGMGT_2 */
	bool stdby_ya; 		/* PWR_MGMGT_2 */
	bool stdby_za; 		/* PWR_MGMGT_2 */
	bool stdby_xg; 		/* PWR_MGMGT_2 */
	bool stdby_yg; 		/* PWR_MGMGT_2 */
	bool stdby_zg; 		/* PWR_MGMGT_2 */
	bool fifo_en;		/* USER_CTRL */
	bool i2c_mst_en; 	/* USER_CTRL */
	bool i2c_if_dis;	/* USER_CTRL */
	bool temp_fifo_en;  	/* FIFO_EN */
	bool accel_fifo_en; 	/* FIFO_EN */
	bool xg_fifo_en;  	/* FIFO_EN */
	bool yg_fifo_en;  	/* FIFO_EN */
	bool zg_fifo_en;  	/* FIFO_EN */
	bool slv0_fifo_en;  	/* FIFO_EN */
	bool slv1_fifo_en;  	/* FIFO_EN */
	bool slv2_fifo_en;  	/* FIFO_EN */
	bool slv3_fifo_en;  	/* I2C_MST_CTRL */
	bool slv4_fifo_en;  	/* I2C_MST_CTRL */
	bool fsync_int_en;  	/* INT_PIN_CFG	__not_supported__ */
	bool fifo_oflow_en;  	/* INT_ENABLE */
	bool i2c_mst_int_en;  	/* INT_ENABLE */
	bool data_rdy_en;  	/* INT_ENABLE */
};

const struct mpu_cfg mpu6050_defcfg = {
	.cfg =	{
		{ PWR_MGMT_1,   0x03},	/* power on, temp enabled, clock gyro_z */
		{ PWR_MGMT_2,   0x00},	/* no standby, full on			*/
		{ CONFIG,       0x00},	/* dlpf off				*/
		{ SMPLRT_DIV,   0x4F},	/* divisor = 80(1+79), rate = 100	*/
		{ ACCEL_CONFIG, 0x00},	/* +-2g 				*/
		{ GYRO_CONFIG,  0x00},	/* +-250 deg/s 				*/
		{ USER_CTRL,    0x60},	/* fifo enabled, aux i2c master mode	*/
		{ FIFO_EN,  	0xF8},	/* temp, accel, gyro buffered		*/
		{ INT_PIN_CFG,  0x00},	/* interrupts disabled			*/
		{ INT_ENABLE,   0x00},	/* interrupts disabled			*/
	}
};

const struct mpu_cfg mpu6050_stcfg = {
	.cfg =	{
		{ PWR_MGMT_1,   0x00},	/* power on, temp enable, int. clk 	*/
		{ PWR_MGMT_2,   0x00},	/* no standby, full on			*/
		{ CONFIG,       0x01},	/* dlpf 1 				*/
		{ SMPLRT_DIV,   0x00},	/* divisor =0 rate = 1000		*/
		{ ACCEL_CONFIG, 0x18},	/* +-16g 				*/
		{ GYRO_CONFIG,  0x00},	/* +-250 deg/s 				*/
		{ USER_CTRL,    0x00},	/* fifo disabled, i2c master disabled	*/
		{ FIFO_EN,  	0x00},	/* no sensors				*/
		{ INT_PIN_CFG,  0x00},	/* interrupts disabled			*/
		{ INT_ENABLE,   0x00},	/* interrupts disabled			*/
	}
};
