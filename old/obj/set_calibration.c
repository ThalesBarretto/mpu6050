/*
 * Apply calibration gain and calibration_offsets on data
 */
void mpu_set_calibration(int *fd)
{
	for (int i=1; i < raw[0]; i++) {
		data[i][0] -= calibration_offsets[i];
		data[i][0] *= calibration_gains[i];
		}
}
