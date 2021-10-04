/*
 * Print calibration in tabular format
 */
void mpu_print_calibration(int *fd)
{
	printf( "=================================================\n"  \
		"       Calibration results\n" \
		"-------------------------------------------------\n" \
		"SENSOR        %-13s\t%-13s\n", \
		"Offset", "Gain" );

	if (a_en == 1 ) {
		printf(	\
		"Ax            %+06.10f\t%+06.10f\n"  \
	        "Ay            %+06.10f\t%+06.10f\n"  \
		"Az            %+06.10f\t%+06.10f\n", \
		*Ax_offset, *Ax_gain, \
		*Ay_offset, *Ay_gain, \
		*Az_offset, *Az_gain);\
	}
	if (t_en == 1 ) {
		printf( \
		"Thermo        %+06.10f\t%+06.10f\n", \
		*Thermo_offset, *Thermo_gain);
	}
	if (g_en == 1) {
		printf( \
		"Gx            %+06.10f\t%+06.10f\n" \
	        "Gy            %+06.10f\t%+06.10f\n" \
	        "Gz            %+06.10f\t%+06.10f\n", \
		*Gx_offset, *Gx_gain, \
		*Gy_offset, *Gy_gain, \
		*Gz_offset, *Gz_gain );
	}
	printf(	"-------------------------------------------------\n" );

}
