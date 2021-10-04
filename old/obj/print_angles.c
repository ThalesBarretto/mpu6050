/*
 * print current Yaw, Pitch, Roll.
 */
void mpu_print_angles(int *fd)
{
	printf("A:y%+03.02fp%+03.02fr:%+03.02ft%+03.02f ", ypr_accel[1], ypr_accel[2], ypr_accel[3], rho);
	printf("G:Y%+03.02fP%+03.02fR:%+03.02f", ypr_gyro[1], ypr_gyro[2], ypr_gyro[3]);
}
