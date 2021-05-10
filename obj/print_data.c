/*
 * Print scaled data in tabular format
 */
void mpu_print_data(int *fd)
{
//	printf("|A|%+2.3f |G|%+2.3f", A_magnitude, G_magnitude);
	if (a_en == 1)
		printf(	" Ax%+06.10f Ay%+06.10f Az%+06.10f", *Ax, *Ay, *Az);
	if (t_en == 1)
		printf( " Tp: %+06.10f", *Thermo);
	if (g_en == 1)
		printf(	" Gx%+06.10f Gy%+06.10f Gz%+06.10f", *Gx, *Gy, *Gz);
}
