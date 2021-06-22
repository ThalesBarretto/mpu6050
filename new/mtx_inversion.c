/*
 * Inverts a matrix using the Gauss Jordan method.
 * returns 0 on success, 1 on failure.
 */
#include <math.h>
int mtxf_inv(float * A, float * B, int n, float *C)
{
	float (*a)[n] = (float (*)[n])A;
	float (*b)[n] = (float (*)[n])B;
	float (*c)[n] = (float (*)[n])C;

	float det = 1;
	float temp, pivot, factor;

	/* use B as buffer to preserve A */
	int i, j;
	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			c[i][j] = 0;
			b[i][j] = a[i][j];
		}
		c[i][i] = 1;
	}

	int iPass, imx, icol, irow;
	for (iPass = 0; iPass < n; iPass++) {
		/* find the maximum element in the pivot column. */
		imx = iPass;
		for (irow = iPass; irow < n; irow++) {
			if (fabs(b[irow][iPass]) > fabs(b[imx][iPass])) {
				imx = irow;
			}
		}
		/* swap elements of row iPass and row imx */
		if (imx != iPass) {
			for (icol = 0; icol < n; icol++) {
				temp = c[iPass][icol];
				c[iPass][icol] = c[imx][icol];
				c[imx][icol] = temp;
				if (icol >= iPass) {
					temp = b[iPass][icol];
					b[iPass][icol] = b[imx][icol];
					b[imx][icol] = temp;
				}
			}
		}

		/* pivot is at A[iPass][iPass]. */
		pivot = b[iPass][iPass];
		det  *= b[iPass][iPass];
		if (det == 0) {
			return -1; /* singular, non-invertible */
		}

		/* Normalize the pivot row dividing by the pivot element. */
		for (icol = 0; icol < n; icol++) {
			c[iPass][icol] /=  pivot;
			if (icol >= iPass) {
				b[iPass][icol] /= pivot;
			}
		}

		/* Add a multiple of the pivot row to each row to make the
		 * element of A on the pivot column equal to 0.
		 */
		for (irow = 0; irow < n; irow++) {
			if (irow != iPass) {
				factor = b[irow][iPass];
			}
			for (icol = 0; icol < n; icol++){
				if (irow != iPass){
					c[irow][icol] -= factor * c[iPass][icol];
					b[irow][icol] -= factor * b[iPass][icol];
				}
			}
		}
	}
	return 1;
}
