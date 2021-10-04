/*
 * Build GSL Linear Least-square fit from scaled data.
 */
void mpu_print_gsl_lsf(int *fd)
{
	/* 52 characters-wide message */
	printf( "=================================================\n"  \
		"       GSL Least-square fit results\n" \
		"-------------------------------------------------\n" \
		"SENSOR        %-13s\t%-13s\n", \
		"Linear", "Angular" );
	int count =1;
	if (a_en == 1) {
		printf(	\
		"Ax            %+06.10f\t%+06.10f\n"  \
	        "Ay            %+06.10f\t%+06.10f\n"  \
		"Az            %+06.10f\t%+06.10f\n", \
		mpu_gsl_fit_linear_c0[count], mpu_gsl_fit_linear_c1[count], \
		mpu_gsl_fit_linear_c0[count+1], mpu_gsl_fit_linear_c1[count+1], \
		mpu_gsl_fit_linear_c0[count+2], mpu_gsl_fit_linear_c1[count+2]); \
		count += 3;
	}
	if (t_en == 1) { printf( \
		"Thermo %+06.10f\t%+06.10f\n", \
		 *Thermo_mean, *Thermo_variance);
		count++;
	}
	if (g_en == 1) {
		printf( \
		"Gx            %+06.10f\t%+06.10f\n" \
	        "Gy            %+06.10f\t%+06.10f\n" \
	        "Gz            %+06.10f\t%+06.10f\n", \
		mpu_gsl_fit_linear_c0[count], mpu_gsl_fit_linear_c1[count], \
		mpu_gsl_fit_linear_c0[count+1], mpu_gsl_fit_linear_c1[count+1], \
		mpu_gsl_fit_linear_c0[count+2], mpu_gsl_fit_linear_c1[count+2]); \
		count += 3;
	}
	printf("-------------------------------------------------\n" );

}
