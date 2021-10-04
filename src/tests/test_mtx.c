#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <limits.h>
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <getopt.h>
#define NOT_NULL (&s != NULL)
#define IS_NULL (&s == NULL)
#include "dev_mpu_mtx.h"

#ifndef MPU_MAXLINE
#define MPU_MAXLINE 1024
#endif
#define PI (3.1415926535897932384L)
#define torad(X) (((X)*PI)/180.0L) 
#define todeg(X) (((X)*180.0L)/PI) 

void mtx_xyz_etr_example(void); 
void mtx_rot_example2(void);
void mtx_rot_example2(void);

void mtx_rot_xyz_example(void);

int main(int argc, char **argv)
{
	mtx_rot_xyz_example();
	mtx_rot_example2();
}
/* This example shows the use of rotation matrices and euler angles.
 *
 * First, build an 4x4 Identity matrix called 'eyem'.
 *
 * Second, given a vector of euler angles, get the rotation matrixes.
 * 	-By default you should provide angles in radians.
 * 	Otherwise, use the flag MTX_ROT_DEG.
 * 
 * 	The angle sequence is PHI, THETA, PSI
 * 	The matrix is obtained from the sequence YAW, PITCH, ROLL.
 *
 *
 *
 */
void mtx_rot_example2(void)
{
	float *euler = (float *)calloc(100,sizeof(float));
	float *quatr = (float *)calloc(100,sizeof(float));
	float *rotmx = (float *)calloc(100,sizeof(float));
	float *bufm = (float *)calloc(100,sizeof(float));
	float *gyrm = (float *)calloc(100,sizeof(float));

	float *eyem = (float *)calloc(100,sizeof(float));
	mtx_eye(3, 3, eyem);

	/* lying flat, straight */
	printf("Euler (XYZ) to rotation matrix 1 - flat\n");
	euler[0] = 0; /* PHI - around x'' axis */
	euler[1] = 0; /* THE - around y' axis */
	euler[2] = 0; /* PSI - around z axis */
	mtx_rot_xyz_etr(MTX_ROT_DEG, euler, rotmx);
	mtx_pri(rotmx, 3, 3);
	printf("R*I\n");
	mtx_mul(rotmx, eyem, 3, 3, 3, bufm);
	mtx_pri(bufm, 3, 3);

	/* This will test the ETR in sequence XYZ */
	printf("Euler to rotation matrix 2 - 90 degress x\n");
	euler[0] = 90; /* phi = 90 degrees */
	euler[1] = 0;
	euler[2] = 0;
	mtx_rot_xyz_etr(MTX_ROT_DEG, euler, bufm);
	mtx_pri(bufm, 3, 3);
	printf("Euler to rotation matrix 3 - 90 degress y\n");
	euler[0] = 0;
	euler[1] = 90; /* the = 90 degrees */
	euler[2] = 0;
	mtx_rot_xyz_etr(MTX_ROT_DEG, euler, bufm);
	mtx_pri(bufm, 3, 3);
	printf("Euler to rotation matrix 3 - 90 degress z\n");
	euler[0] = 0;
	euler[1] = 0;
	euler[2] = 90; /*psi = 90 degrees */
	mtx_rot_xyz_etr(MTX_ROT_DEG, euler, bufm);
	mtx_pri(bufm, 3, 3);

	/* now, with a more complex example */
	euler[0] = 30; /*phi = 30 degrees */
	euler[1] = 60; /*psi = 60 degrees */
	euler[2] = 45; /*psi = 45 degrees */
	printf("ETR - Euler(30,60,45) in degrees\n");
	mtx_pri(euler, 3, 1);

	printf("ETR - Rotation matrix for Euler(30,60,45)\n");
	mtx_rot_xyz_etr(MTX_ROT_DEG, euler, rotmx);
	mtx_pri(rotmx, 3, 3);

	printf("RTE - Euler angles for Rotation matrix\n");
	mtx_rot_xyz_rte(rotmx, MTX_ROT_DEG, bufm);
	mtx_pri(bufm, 3, 1);

	printf("ETQ - Quaternion for Euler(30,60,45)\n");
	mtx_rot_xyz_etq(MTX_ROT_DEG, euler, quatr);
	mtx_pri(quatr, 4, 1);

	printf("QTE - Euler angles from Quaternion\n");
	mtx_rot_xyz_qte(quatr, MTX_ROT_DEG, bufm);
	mtx_pri(bufm, 3, 1);


	gyrm[0] = 10;
	gyrm[1] = 20;
	gyrm[2] = 30;
	printf("CPM - for GYRO(10,20,30)\n");
	mtx_rot_xyz_cpm(gyrm, &bufm[50]);
	mtx_pri(&bufm[50], 3, 3);
	printf("CP2- for GYRO(10,20,30)\n");
	mtx_rot_xyz_cp2(gyrm, &bufm[50]);
	mtx_pri(&bufm[50], 3, 3);
}
void mtx_rox_example(void)
{
	float A[9]    = {1,2,2,
			 0,2,2,
			 0,0,1};

	float *B = (float *)calloc(100,sizeof(float));
	float *C = (float *)calloc(100,sizeof(float));

	printf("The Matrix A\n");
	mtx_pri(A, 3, 3);
	printf("\n");

	//mtxf_inv(A, 3, C);
	mtx_inv(A, B, 3, C);
	printf("The Matrix A in row echelon form\n");
	mtx_pri(B, 3, 3);
	printf("\n");

	printf("The Matrix A inverse\n");
	mtx_pri(C, 3, 3);
	printf("\n");
	long double q = torad(0);
	printf("Type an angle in degrees:");
	scanf("%Lf", &q);
	q = torad(q);

	printf("The Matrix Rx(%.Lf)\n", todeg(q));
	mtx_roz(q, C);
	mtx_pri(C, 3, 3);
	printf("\n");

	printf("The Matrix Ry(%.Lf)\n", todeg(q));
	mtx_roy(q, C);
	mtx_pri(C, 3, 3);
	printf("\n");

	printf("The Matrix Rz(%.Lf)\n", todeg(q));
	mtx_roz(q, C);
	mtx_pri(C, 3, 3);
	printf("\n");
}

void mtx_rot_xyz_example(void)
{
	float v[3] = {1, 1, 1}; /* 3d vector */
	float *E = (float *)calloc(3,sizeof(float)); 	/* euler angles */
	float *C = (float *)calloc(100,sizeof(float));	/* auxiliary matrix */
	float *R = (float *)calloc(100,sizeof(float));	/* rotation matrix */
	float *w = (float *)calloc(100,sizeof(float));  /* final vector */

	long double q = 0;

	printf("The vector v\n");
	mtx_pri(v, 3, 1);
	printf("\n");

	printf("Type angle sequence (roll,pitch,yaw) in degrees:\n");
	scanf("%f,%f,%f", &E[0],&E[1], &E[2]);
	printf("The angles:\n");
	mtx_pri(E, 3, 1);
	printf("Rotate v bu E ( w = R * v):\n");
	mtx_rot_xyz_etr(MTX_ROT_DEG, E, R);
	printf("Tha matrix R (Rotation Matrix)\n");
	mtx_pri(R, 3, 3);
	mtx_mul(R, v, 3, 3, 1, w);
	mtx_pri(w, 3, 1);
	mtx_zer(3, 1, w);
	printf("\n");

	printf("The Matrix Ry(%.Lf)\n", todeg(q));
	mtx_roy(q, C);
	mtx_pri(C, 3, 3);
	printf("\n");

	printf("The Matrix Rz(%.Lf)\n", todeg(q));
	mtx_roz(q, C);
	mtx_pri(C, 3, 3);
	printf("\n");
}

void mtx_example(void)
{
	long double a[4]    = {  1,  2,   3,  4  };
	long double b[2][2] = { {5,  6}, {7,  8} };

	long double *A = (long double *)a;
	long double *B = (long double *)b;
	long double *C = (long double *)calloc(100,sizeof(long double));
	long double k = 3;

	printf("The Matrix A\n");
	mtx_pri(A, 2, 2);
	printf("Matrix A - column 1\n");
	mtx_pri_col(A, 2, 1);
	printf("Matrix A - column 2\n");
	mtx_pri_col(A, 2, 2);
	printf("Matrix A - row 1\n");
	mtx_pri_row(A, 2, 1);
	printf("Matrix A - row 2\n");
	mtx_pri_row(A, 2, 2);
	printf("Matrix Atranspose\n");
	mtx_trn(A, 2, 2, C);
	mtx_pri(C, 2, 2);

	printf("swap A rows\n");
	mtx_row_swp(A, 2, 1, 2);
	mtx_pri(A, 2, 2);
	printf("from A add row 1 to row 2\n");
	mtx_row_add(A, 2, 1, 2);
	mtx_pri(A, 2, 2);
	printf("from A subtract row 1 from row 2\n");
	mtx_row_add(A, 2, 1, 2);
	mtx_pri(A, 2, 2);
	printf("from A add 3 times row 1 to row 2\n");
	mtx_row_adm(A, 2, 3, 1, 2);
	mtx_pri(A, 2, 2);
	printf("from A add -3 times row 1 from row 2\n");
	mtx_row_adm(A, 2, -3, 1, 2);
	mtx_pri(A, 2, 2);

	printf("swap A columns\n");
	mtx_col_swp(A, 2, 1, 2);
	mtx_pri(A, 2, 2);

	printf("The Matrix B\n");
	mtx_pri(B, 2, 2);

	printf("The zero matrix\n");
	mtx_zer(2, 2, C);
	mtx_pri(C, 2, 2);

	printf("The identity matrix\n");
	mtx_eye(2, 2, C);
	mtx_pri(C, 2, 2);

	printf("C = A + B=\n");
	mtx_add(A, B, 2, 2, C);
	mtx_pri(C, 2, 2);

	printf("C = A - B=\n");
	mtx_sub(A, B, 2, 2, C);
	mtx_pri(C, 2, 2);

	printf("C = A * B\n");
	mtx_mul(A, B, 2, 2, 2, C);
	mtx_pri(C, 2, 2);

	printf("C = A + k\n");
	mtx_add_scl(A, 2, 2, k, C);
	mtx_pri(C, 2, 2);
	printf("C = A - k\n");
	mtx_sub_scl(A, 2, 2, k, C);
	mtx_pri(C, 2, 2);

	printf("C = A * k\n");
	mtx_mul_scl(A, 2, 2, k, C);
	mtx_pri(C, 2, 2);

	printf("C = A / k\n");
	mtx_div_scl(A, 2, 2, k, C);
	mtx_pri(C, 2, 2);
}

void pr_arr(int *p, int m)
{
	for (int i = 0; i < m; i++) {
		printf("p[%d]=%d\n", i, p[i]);
	}


}

void pr_mtx_x(int *w, int m, int n)
{
	int (*p)[n] = (int (*)[n])w;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			printf("p[%d][%d]=%d\n", i, j, p[i][j]);
		}
	}
	printf("\n");
}
void example(void)
{
	int a[4]    = {  1,  2,   3,  4  };
	int b[2][2] = { {5,  6}, {7,  8} };
	int c[4]    = {  9, 10,  11, 12  };
	int (*x)[2] = (int (*)[2])&a;	/* compiler complaint w/o cast */
	int (*y)[2] = b;
	int (*z)[2] = (int (*)[2])c;	/* compiler complaint w/o cast */
	int *w      = (int *)x;		/* compiler complaint w/o cast */
	int *p = a;

	pr_arr(a, 4);
	pr_mtx_x((int*)y, 2, 2);
	printf("a[0]= %d a[1]= %d a[2]= %d a[1]= %d\n",
		       	a[0], a[1], a[2], a[3]);
	printf("p[0]= %d p[1]= %d p[2]= %d p[1]= %d\n",
		       	p[0], p[1], p[2], p[3]);
	printf("x[0][0]= %d x[0][1]=%d x[1][0]=%d x[1][1]=%d\n",
		       	x[0][0], x[0][1], x[1][0], x[1][1]);
	printf("z[0][0]= %d z[0][1]=%d z[1][0]=%d z[1][1]=%d\n",
		       	z[0][0], z[0][1], z[1][0], z[1][1]);
	printf("w[0]= %d w[1]= %d w[2]= %d w[1]= %d\n",
		       	w[0], w[1], w[2], w[3]);

	printf("b[0][0]= %d b[0][1]=%d b[1][0]=%d b[1][1]=%d\n",
		       	b[0][0], b[0][1], b[1][0], b[1][1]);
	printf("y[0][0]= %d y[0][1]=%d y[1][0]=%d y[1][1]=%d\n",
		       	y[0][0], y[0][1], y[1][0], y[1][1]);
	y[0][0] = 5;
       	y[0][1] = 6;
       	y[1][0] = 7;
       	y[1][1] = 8;
	printf("b[0][0]= %d b[0][1]=%d b[1][0]=%d b[1][1]=%d\n",
		       	b[0][0], b[0][1], b[1][0], b[1][1]);
	printf("y[0][0]= %d y[0][1]=%d y[1][0]=%d y[1][1]=%d\n",
		       	y[0][0], y[0][1], y[1][0], y[1][1]);
}

