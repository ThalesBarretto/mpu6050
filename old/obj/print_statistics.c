/*
 * Build statistics from scaled data.
 */
void mpu_print_statistics(int *fd)
{
	/* 52 characters-wide message */
	printf( "=================================================\n"  \
		"       Statistics results\n" \
		"-------------------------------------------------\n" \
		"SENSOR        %-13s\t%-13s\n", \
		"mean", "variance" );

	if (a_en == 1) {
		printf(	\
		"Ax            %+06.10f\t%+06.10f\n"  \
	        "Ay            %+06.10f\t%+06.10f\n"  \
		"Az            %+06.10f\t%+06.10f\n", \
		*Ax_mean, *Ax_variance, \
		*Ay_mean, *Ay_variance, \
		*Az_mean, *Az_variance);\
	}
	if (t_en == 1) { printf( \
		"Thermo %+06.10f\t%+06.10f\n", \
		 *Thermo_mean, *Thermo_variance);
	}
	if (g_en == 1) {
		printf( \
		"Gx            %+06.10f\t%+06.10f\n" \
	        "Gy            %+06.10f\t%+06.10f\n" \
	        "Gz            %+06.10f\t%+06.10f\n", \
		*Gx_mean, *Gx_variance, \
		*Gy_mean, *Gy_variance, \
		*Gz_mean, *Gz_variance );
	}
	printf("-------------------------------------------------\n" );

}
