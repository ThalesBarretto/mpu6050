#include "imu_get_data_mpu6050.h"

/* get_data_mpu6050 specific */
void imu_get_data_mpu6050(struct IMU *imu)
{
#define mpu ((struct mpu_dev *)imu->dev)

	mpu_ctl_fifo_data(mpu);
	mpu_ctl_fix_axis(mpu);
	imu->Ts = mpu->st;
	/* way faster than a generic solution */
	imu->acc[0] =  (*mpu->Ax);
	imu->acc[1] = -(*mpu->Ay);
	imu->acc[2] = -(*mpu->Az);
	imu->gyr[0] =  d2r(*mpu->Gx);
	imu->gyr[1] = -d2r(*mpu->Gy);
	imu->gyr[2] = -d2r(*mpu->Gz);
#undef mpu
}
