#include "dev_mpu_pub.h"
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#define MDEBUG printf("%3d %-12s\n", __LINE__, __func__);


int main(int argc, char *argv[])
{
	struct mpu_dev * dev = NULL;
	mpu_init("/dev/i2c-1", MPU6050_ADDR, &dev);
	assert(dev != NULL);

	mpu_ctl_selftest(dev);


	return 0;
}

int mpu_unit_test(struct mpu_dev *dev)
{
	printf("TESTING: read data test...\n");
	int16_t thermo = 0;
	mpu_read_data(dev, TEMP_OUT_H, &thermo);
	long double temperature ;
	temperature = (((long double)thermo/340.0L) + 36.53L);
	printf("TEMPERATURE: %d %2.18Lf\n", thermo, temperature);

	printf("TESTING: diagnose test...\n");
	mpu_diagnose(dev);
	printf("TESTING: temperature false test...\n");
	mpu_ctl_temperature(dev, false);
	mpu_print_data(dev, 3);
	printf("TESTING: temperature true test...\n");
	mpu_ctl_temperature(dev, true);
	mpu_print_data(dev, 3);
	printf("TESTING: clocksource 0 test...\n");
	mpu_ctl_clocksource(dev, 0);
	mpu_print_data(dev, 3);
	printf("TESTING: clocksource 1 test...\n");
	mpu_ctl_clocksource(dev, 1);
	mpu_print_data(dev, 3);
	printf("TESTING: clocksource 2 test...\n");
	mpu_ctl_clocksource(dev, 2);
	mpu_print_data(dev, 3);
	printf("TESTING: clocksource 3 test...\n");
	mpu_ctl_clocksource(dev, 3);
	mpu_print_data(dev, 3);
	printf("TESTING: dlpf 0 test...\n");
	mpu_ctl_dlpf(dev,0);
	mpu_print_data(dev, 3);
	printf("TESTING: dlpf 1 test...\n");
	mpu_ctl_dlpf(dev,1);
	mpu_print_data(dev, 3);
	printf("TESTING: dlpf 2 test...\n");
	mpu_ctl_dlpf(dev,2);
	mpu_print_data(dev, 3);
	printf("TESTING: dlpf 3 test...\n");
	mpu_ctl_dlpf(dev,3);
	mpu_print_data(dev, 3);
	printf("TESTING: dlpf 4 test...\n");
	mpu_ctl_dlpf(dev,4);
	mpu_print_data(dev, 3);
	printf("TESTING: dlpf 5 test...\n");
	mpu_ctl_dlpf(dev,5);
	mpu_print_data(dev, 3);
	printf("TESTING: fifo reset test...\n");
	mpu_ctl_fifo_reset(dev);
	mpu_ctl_fifo_flush(dev);
	mpu_print_data(dev, 3);
	printf("TESTING: fifo disable test...\n");
	mpu_ctl_fifo_disable(dev);
	mpu_ctl_fifo_flush(dev);
	mpu_print_data(dev, 3);
	printf("TESTING: fifo enable test...\n");
	mpu_ctl_fifo_enable(dev);
	mpu_ctl_fifo_flush(dev);
	mpu_print_data(dev, 3);
	printf("TESTING: fifo disable accel test...\n");
	mpu_ctl_fifo_disable_accel(dev);
	mpu_ctl_fifo_flush(dev);
	mpu_print_data(dev, 3);
	printf("TESTING: fifo disable gyro test...\n");
	mpu_ctl_fifo_disable_gyro(dev);
	mpu_ctl_fifo_flush(dev);
	mpu_print_data(dev, 3);
	printf("TESTING: fifo disable temp test...\n");
	mpu_ctl_fifo_disable_temp(dev);
	mpu_ctl_fifo_flush(dev);
	mpu_print_data(dev, 3);
	printf("TESTING: fifo enable accel test...\n");
	mpu_ctl_fifo_enable_accel(dev);
	mpu_ctl_fifo_flush(dev);
	mpu_print_data(dev, 3);
	printf("TESTING: fifo enable gyro test...\n");
	mpu_ctl_fifo_enable_gyro(dev);
	mpu_ctl_fifo_flush(dev);
	mpu_print_data(dev, 3);
	printf("TESTING: fifo enable temp test...\n");
	mpu_ctl_fifo_enable_temp(dev);
	mpu_ctl_fifo_flush(dev);
	mpu_print_data(dev, 3);
	printf("TESTING: selftest enable accel test...\n");
	mpu_ctl_selftest_enable_accel(dev);
	mpu_print_data(dev, 3);
	printf("TESTING: selftest disable accel test...\n");
	mpu_ctl_selftest_disable_accel(dev);
	mpu_print_data(dev, 3);
	printf("TESTING: selftest enable gyro test...\n");
	mpu_ctl_selftest_enable_gyro(dev);
	mpu_print_data(dev, 3);
	printf("TESTING: selftest disable gyro test...\n");
	mpu_ctl_selftest_disable_gyro(dev);
	mpu_print_data(dev, 3);
	printf("TESTING: accel_range 2 test...\n");
	mpu_ctl_accel_range(dev, 2);
	mpu_print_data(dev, 3);
	printf("TESTING: accel_range 4 test...\n");
	mpu_ctl_accel_range(dev, 4);
	mpu_print_data(dev, 3);
	printf("TESTING: accel_range 8 test...\n");
	mpu_ctl_accel_range(dev, 8);
	mpu_print_data(dev, 3);
	printf("TESTING: accel_range 16 test...\n");
	mpu_ctl_accel_range(dev, 16);
	mpu_print_data(dev, 3);
	printf("TESTING: gyro_range 250 test...\n");
	mpu_ctl_gyro_range(dev, 250);
	mpu_print_data(dev, 3);
	printf("TESTING: gyro_range 500 test...\n");
	mpu_ctl_gyro_range(dev, 500);
	mpu_print_data(dev, 3);
	printf("TESTING: gyro_range 1000 test...\n");
	mpu_ctl_gyro_range(dev, 1000);
	mpu_print_data(dev, 3);
	printf("TESTING: gyro_range 2000 test...\n");
	mpu_ctl_gyro_range(dev, 2000);
	mpu_print_data(dev, 3);

	return 0;
}
