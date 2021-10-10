#ifdef __cplusplus
	extern "C" {
#endif

#ifndef _IMU_GET_DATA_MPU6050_H_
#define _IMU_GET_DATA_MPU6050_H_

#include "dev_mpu.h"
#include "dev_mpu_mtx.h"
#include "dev_mpu_mtx_pri.h"
#include "dev_mpu_mtx_rot.h"
#include "imu.h"

/* The "get_data" connects the driver data to the system.
 * Every device will have its own driver and 'get_data'
 */
void imu_get_data_mpu6050(struct IMU *imu);

#endif /* _IMU_GET_DATA_MPU6050_H_ */

#ifdef __cplusplus
	}
#endif
