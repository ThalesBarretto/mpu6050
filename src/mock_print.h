#ifdef __cplusplus
	extern "C" {
#endif
#ifndef _MOCK_PRINT_H_
#define _MOCK_PRINT_H_
#include <libmpu6050/mpu6050_core.h>

void mpu_print_all   (struct mpu_dev *dev, char *msg, char *buf);
void mpu_print_data  (struct mpu_dev *dev, int times);
void mpu_print_bias  (struct mpu_dev *dev);

#endif /* _MOCK_PRINT_H_ */
#ifdef __cplusplus
	}
#endif
