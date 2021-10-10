#include "imu.h"
#include "dev_mpu_mtx_rot_ypr.h"
//TODO: figure out how to do this
void imu_estimate(struct IMU *imu)
{
	long double Omega[3];	// Euler angles from accel
	long double Phi_old[3];	// Euler angles from accel
	long double Phi_dot[3];	// Euler angles from accel
	long double Phi_inc[3]; // Euler angle increment over period T
	long double R_old[9];	// Rotation matrix for old attitude
	long double R_inc[9];	// Rotation matrix for increment
	long double R_new[9];	// Rotation matrix for new attitude
	long double Phi_gyr[3];	// Euler new angles

	Phi_old[0] = imu->fr->Phi[0];
	Phi_old[1] = imu->fr->Phi[1];
	Phi_old[2] = imu->fr->Phi[2];
	mtx_rot_ypr_etr(Phi_old, R_old);
	Omega[0] = imu->gyr[0];
	Omega[1] = imu->gyr[1];
	Omega[2] = imu->gyr[2];

	mtx_rot_ypr_wtp	(Phi_old, Omega, Phi_dot);
	mtx_mul_scl(Phi_dot, 3, 1, imu->Ts, Phi_inc);
	mtx_rot_ypr_etr	(Phi_inc, R_inc);
	mtx_mul(R_inc, R_old, 3,3,3, R_new);
	mtx_rot_ypr_rte(R_new, Phi_gyr);

//	printf("Ts: %+6lf ", imu->Ts);
	printf("phi_old: %+6Lf ", r2d(Phi_old[0]));
	printf("the_old: %+6Lf ", r2d(Phi_old[1]));
	printf("psi_old: %+6Lf ", r2d(Phi_old[2]));
	printf("phi_dot: %+6Lf ", r2d(Phi_dot[0]));
	printf("the_dot: %+6Lf ", r2d(Phi_dot[1]));
	printf("psi_dot: %+6Lf ", r2d(Phi_dot[2]));
//	printf("phi_inc: %+6Lf ", r2d(Phi_inc[0]));
//	printf("the_inc: %+6Lf ", r2d(Phi_inc[1]));
//	printf("psi_inc: %+6Lf ", r2d(Phi_inc[2]));
	printf("G_phi: %+6Lf ", r2d(Phi_gyr[0]));
	printf("G_the: %+6Lf ", r2d(Phi_gyr[1]));
	printf("G_psi: %+6Lf ", r2d(Phi_gyr[2]));

	/* angles from accelerometer */
	long double Phi_acc[3];	// Euler angles from accel
	Phi_acc[0] = atan2l( imu->acc[1], imu->acc[2]);
	Phi_acc[1] = atan2l(-imu->acc[0], imu->acc[2]);
	Phi_acc[2] = 0;
	imu->fr->Phi[0] = Phi_acc[0];
	imu->fr->Phi[1] = Phi_acc[1];
	imu->fr->Phi[2] = Phi_acc[2];
	//printf("A_phi: %+6lf ", r2d(Phi_acc[0]));
	//printf("A_the: %+6lf ", r2d(Phi_acc[1]));
	//printf("A_psi: %+6lf ", r2d(Phi_acc[2]));
	printf("\n");
}

//TODO: put declaration in header
//void imu_print_Phi(struct IMU *imu)
//{
//	long double phi = r2d(imu->fr->Phi[0]);
//	long double the = r2d(imu->fr->Phi[1]);
//	long double psi = r2d(imu->fr->Phi[2]);
//	double P = imu->fr->Omega[0];
//	double Q = imu->fr->Omega[1];
//	double R = imu->fr->Omega[2];
//	printf("Ts: %+6lf ", imu->Ts);
//	printf("Ax: %+6lf ", imu->acc[0]);
//	printf("Ay: %+6lf ", imu->acc[1]);
//	printf("Az: %+6lf ", imu->acc[2]);
//	printf("Gx: %+6lf ", imu->gyr[0]);
//	printf("Gy: %+6lf ", imu->gyr[1]);
//	printf("Gz: %+6lf ", imu->gyr[2]);
//	//printf("P: %+6lf ", P);
//	//printf("Q: %+6lf ", Q);
//	//printf("R: %+6lf ", R);
//	printf("phi: %+6lf ", phi);
//	printf("the: %+6lf ", the);
//	printf("psi: %+6lf ", psi);
//	printf("\n");
//}
