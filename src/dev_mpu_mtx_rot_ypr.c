#include "dev_mpu_mtx_rot_ypr.h"

/* See NOTES at the end
 * MATRIX ROTATIONS - YPR Sequencei
 *
 * Functions:
 * 	ETR - Euler angles to rotation matrix
 *	ETQ - Euler angles to quaternion
 * 	RTE - Rotation matrix to Euler angles
 *	RTQ - Rotation matrix to quaternion
 *	QTE - Quaternion to Euler angles
 *	QTR - Quaternion to rotation matrix
 *	WTP - Body angle rates to Euler angle rates
 *TODO	WTQ - Body angle rates to Quaternions rates
 *TODO	BTR - body to reference (reverse) Rotation matrix from ypr Euler angles 
 *TODO	BTQ - body to reference Quaterion from (already reverse) Rotation matrix
 *TODO	BTE - ypr Euler angles from body to reference (reverse) Rotation matrix
 *TODO	ATE - Accelerometer (stationary) to Euler angle
 *
 * References:
 *  HENDERSON, D. M.  (Eq. 10 ZYX(1,2,3)- 1=yaw,2=pitch,3=roll)
 *  	M321		(eq. 10) ypr EULER to REVERSE MATRIX
 *  	q321		(eq. 10) REVERSE MATRIX to REVERSE QUATERNION
 *  	E321		(eq. 10) REVERSE MATRIX to ypr EULER ANGLES
 *
 *  JOHSON, et al.
 * 	Cfrd/ned	(eq. 1.3-10) EULER  to MATRIX
 * 	PHI(Cfrd/ned)	(eq. 1.3-11) MATRIX to EULER
 *	PHIdot		(eq. 1.4-4)  EULER, GYRO to EULER RATES
 *
 *  DIEBEL, James.
 * 	R_123(phi,theta,psi)	(eq.287) EULER to MATRIX 
 * 	L{R_123(phi,theta,psi)}	(eq.288) EULER to MATRIX LINEARIZED
 * 	q_123(phi,theta,psi) 	(eq.297) EULER to QUATERNION 
 * 	q_n(Rq)	 		(eq.145) MATRIX to QUATERNION
 * 	u_123(R)	 	(eq.289) MATRIX to EULER 
 * 	u_123(R_q(q))	 	(eq.290) QUATERNION to EULER 
 * 	R_q(q)	 		(eq.125) QUATERNION to MATRIX 
 */

/* ETR - Euler angles to rotation matrix
 *  JOHNSON:	Cfrd/ned		(eq. 1.3-10) EULER  to MATRIX
 *  DIEBEL:	R_123(phi,theta,psi)	(eq.287) EULER to MATRIX 
 */
#define MTX_ROT_YPR_ETR							\
	c[0][0] = ( c_the * c_psi );					\
	c[0][1] = ( c_the * s_psi );					\
	c[0][2] = (-s_the );						\
	c[1][0] = ( s_phi * s_the * c_psi) - ( c_phi * s_psi );		\
	c[1][1] = ( s_phi * s_the * s_psi) + ( c_phi * c_psi );		\
	c[1][2] = ( s_phi * c_the );					\
	c[2][0] = ( c_phi * s_the * c_psi) + ( s_phi * s_psi );		\
	c[2][1] = ( c_phi * s_the * s_psi) - ( s_phi * c_psi );		\
	c[2][2] = ( c_phi * c_the );

/* ETQ - Euler angles to quaternion
 *  DIEBEL:	q_123(phi,theta,psi) 	(eq.297) EULER to QUATERNION 
 */
#define MTX_ROT_YPR_ETQ							\
	C[0] =  cphi2*cthe2*cpsi2 + sphi2*sthe2*spsi2;			\
	C[1] = -cphi2*sthe2*spsi2 + cthe2*cpsi2*sphi2;			\
	C[2] =  cphi2*cpsi2*sthe2 + sphi2*cthe2*spsi2;			\
	C[3] =  cphi2*cthe2*spsi2 - sphi2*cpsi2*sthe2;

/* RTE - Rotation matrix to Euler angles
 *  JOHNSON:	PHI(Cfrd/ned)	(eq. 1.3-11) MATRIX to EULER
 *  DIEBEL:	u_123(R) 	(eq.289)     MATRIX to EULER 
 */ 
#define MTX_ROT_YPR_RTE							\
	C[0] =   atan2(a[1][2], a[2][2]); 				\
	C[1] =  -asin( a[0][2]);					\
	C[2] =   atan2(a[0][1], a[0][0]);

/* RTQ - Rotation matrix to quaternion
 *  DIEBEL:	q_n(Rq)	 		(eq.145) MATRIX to QUATERNION
 */
#define MTX_ROT_YPR_RTQ									\
	int form = -1;	/* form selection (eq. 145) */					\
	if 	((a[1][1]>-a[2][2])&&(a[0][0]>-a[1][1])&&(a[0][0]>-a[2][2])){form = 0;}	\
	else if ((a[1][1]<-a[2][2])&&(a[0][0]> a[1][1])&&(a[0][0]> a[2][2])){form = 1;}	\
	else if ((a[1][1]> a[2][2])&&(a[0][0]< a[1][1])&&(a[0][0]<-a[2][2])){form = 2;} \
	else if ((a[1][1]< a[2][2])&&(a[0][0]<-a[1][1])&&(a[0][0]< a[2][2])){form = 3;} \
	switch (form) {									\
		case 0:	/* form 0  (eq. 141)	*/					\
			rterm = sqrt(1+ a[0][0] + a[1][1] + a[2][2]);			\
			C[0] = 			   (rterm)/2.L;				\
			C[1] = ((a[1][2] - a[2][1])/rterm)/2.L;				\
			C[2] = ((a[2][0] - a[0][2])/rterm)/2.L;				\
			C[3] = ((a[0][1] - a[1][0])/rterm)/2.L;				\
			break;								\
		case 1:	/* form 1 (eq. 142)	*/					\
			rterm = sqrt(1+ a[0][0] - a[1][1] - a[2][2]);			\
			C[0] = ((a[1][2] - a[2][1])/rterm)/2.L;				\
			C[1] = 			   (rterm)/2.L;				\
			C[2] = ((a[0][1] + a[1][0])/rterm)/2.L;				\
			C[3] = ((a[2][0] + a[0][2])/rterm)/2.L;				\
			break;								\
		case 2:	/* form 2 (eq. 143)	*/					\
			rterm = sqrt(1- a[0][0] + a[1][1] - a[2][2]); 			\
			C[0] = ((a[2][0] - a[0][2])/rterm)/2.L;				\
			C[1] = ((a[0][1] + a[1][0])/rterm)/2.L;				\
			C[2] = 			   (rterm)/2.L;				\
			C[3] = ((a[1][2] + a[2][1])/rterm)/2.L;				\
			break;								\
		case 3:	/* form 3 (eq. 144)	*/					\
			rterm = sqrt(1- a[0][0] - a[1][1] + a[2][2]); 			\
			C[0] = ((a[0][1] - a[1][0])/rterm)/2.L;				\
			C[1] = ((a[2][0] + a[0][2])/rterm)/2.L;				\
			C[2] = ((a[1][2] + a[2][1])/rterm)/2.L;				\
			C[3] = 			   (rterm)/2.L;				\
			break;								\
	}

/* QTE - Quaternion to Euler angles
 *  DIEBEL:	u_123(R_q(q))	 	(eq.290) QUATERNION to EULER 
 */
#define MTX_ROT_YPR_QTE							\
	C[0] =  atan2((2*A[2]*A[3]) + (2*A[0]*A[1]), 			\
			squ(A[3]) - squ(A[2]) - squ(A[1]) + squ(A[0]));	\
	C[1] = -asin((2*A[1]*A[3]) - (2*A[0]*A[2]));			\
	C[2] =  atan2((2*A[1]*A[2]) + (2*A[0]*A[3]), 			\
			squ(A[1]) + squ(A[0]) - squ(A[3]) - squ(A[2])); 

/* QTR - Quaternion to rotation matrix
 *  DIEBEL:	R_q(q)	 		(eq.125) QUATERNION to MATRIX 
 */
#define MTX_ROT_YPR_QTR							\
	c[0][0] = squ(A[0]) + squ(A[1]) -  squ(A[2]) - squ(A[3]);	\
	c[0][1] = 2*(A[1])*(A[2]) + 2*(A[0])*(A[3]);			\
	c[0][2] = 2*(A[1])*(A[3]) - 2*(A[0])*(A[2]);			\
	c[1][0] = 2*(A[1])*(A[2]) - 2*(A[0])*(A[3]);			\
	c[1][1] =  squ(A[0]) - squ(A[1]) +  squ(A[2]) - squ(A[3]);	\
	c[1][2] = 2*(A[2])*(A[3]) + 2*(A[0])*(A[1]);			\
	c[2][0] = 2*(A[1])*(A[3]) + 2*(A[0])*(A[2]);			\
	c[2][1] = 2*(A[2])*(A[3]) - 2*(A[0])*(A[1]);			\
	c[2][2] =  squ(A[0]) - squ(A[1]) -  squ(A[2]) + squ(A[3]);

/* WTP - Body angle rates to Euler angle rates
 *  JOHNSON:	PHIdot		(eq. 1.4-4)  EULER, GYRO to EULER RATES
 */
#define MTX_ROT_YPR_WTP							\
	C[0] = B[0] + 	  (B[1]*sphi*tthe) + (B[2]*cphi*tthe) ;		\
	C[1] = 		  (B[1]*cphi) 	   - (B[2]*sphi) ; 		\
	C[2] = B[0]*sphi +(B[2]*cphi/cthe);


/* BTR - Euler angles to reverse rotation matrix
 *  HENDERSON: MZYX(3,2,1) (seq. 10)  (1=phi,2=theta,3=psi)
 */
#define MTX_ROT_YPR_BTR							\
	c[0][0] = ( c_psi * c_the );					\
	c[0][1] = ( s_phi * s_the * c_psi) - ( s_psi * c_phi);		\
	c[0][2] = ( s_the * c_phi * c_psi) + ( s_phi * s_psi);		\
	c[1][0] = ( s_psi * c_the );					\
	c[1][1] = ( s_phi * s_psi * s_the) + ( c_phi * c_psi);		\
	c[1][2] = (-s_phi * c_psi) + ( s_psi * s_the * c_phi);		\
	c[2][0] = (-s_the);						\
	c[2][1] = ( s_phi * c_the);					\
	c[2][2] = ( c_phi * c_the);

/* BTQ - Euler angles to reverse quaternion
 *  HENDERSON: MZYX(3,2,1) (seq. 10)  (1=phi,2=theta,3=psi)
 */
#define MTX_ROT_YPR_BTQ							\
	C[0] =  sphi2*sthe2*spsi2 + cphi2*cthe2*cpsi2;			\
	C[1] = -sphi2*sthe2*cpsi2 + spsi2*cphi2*cthe2;			\
	C[2] =  sphi2*spsi2*cthe2 + sthe2*cphi2*cpsi2;			\
	C[3] =  sphi2*cthe2*cpsi2 - sthe2*spsi2*cphi2;

/* BTE - Reverse rotation matrix to Euler angles
 *  HENDERSON: MZYX(3,2,1) (seq. 10)  (1=phi,2=theta,3=psi)
 */ 
#define MTX_ROT_YPR_BTE							\
	C[0] =   atan( a[1][0]/a[0][0]); 				\
	C[1] =   atan(-a[2][0]/(sqrt(1- squ(a[2][0]))));		\
	C[2] =   atan( a[2][1]/a[2][2]);

void  mtxf_rot_ypr_etr(const float * const A,  float *C)
{
	float (*c)[3] = (float (*)[3])C;
	float s_phi = sinf(A[0]);
	float c_phi = cosf(A[0]);
	float s_the = sinf(A[1]);
	float c_the = cosf(A[1]);
	float c_psi = cosf(A[2]);
	float s_psi = sinf(A[2]);
	MTX_ROT_YPR_ETR;
}
void mtxlf_rot_ypr_etr(const double * const  A, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	double s_phi = sin(A[0]);
	double c_phi = cos(A[0]);
	double s_the = sin(A[1]);
	double c_the = cos(A[1]);
	double c_psi = cos(A[2]);
	double s_psi = sin(A[2]);
	MTX_ROT_YPR_ETR;
}
void mtxLf_rot_ypr_etr(const long double * const A, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	long double s_phi = sinl(A[0]);
	long double c_phi = cosl(A[0]);
	long double s_the = sinl(A[1]);
	long double c_the = cosl(A[1]);
	long double c_psi = cosl(A[2]);
	long double s_psi = sinl(A[2]);
	MTX_ROT_YPR_ETR;
}
void  mtxf_rot_ypr_etq(const float * const A,  float *C)
{
	float cphi2 = cosf(A[0]/2);
	float sphi2 = sinf(A[0]/2);
	float cthe2 = cosf(A[1]/2);
	float sthe2 = sinf(A[1]/2);
	float cpsi2 = cosf(A[2]/2);
	float spsi2 = sinf(A[2]/2);
	MTX_ROT_YPR_ETQ;
}
void mtxlf_rot_ypr_etq(const double * const  A, double *C)
{
	double cphi2 = cos(A[0]/2);
	double sphi2 = sin(A[0]/2);
	double cthe2 = cos(A[1]/2);
	double sthe2 = sin(A[1]/2);
	double cpsi2 = cos(A[2]/2);
	double spsi2 = sin(A[2]/2);
	MTX_ROT_YPR_ETQ;
}
void mtxLf_rot_ypr_etq(const long double * const A, long double *C)
{
	float cphi2 = cosl(A[0]/2);
	float sphi2 = sinl(A[0]/2);
	float cthe2 = cosl(A[1]/2);
	float sthe2 = sinl(A[1]/2);
	float cpsi2 = cosl(A[2]/2);
	float spsi2 = sinl(A[2]/2);
	MTX_ROT_YPR_ETQ;
}
void  mtxf_rot_ypr_rte(const float * const A, float *C)
{
	float (*a)[3] = (float (*)[3])A;
	MTX_ROT_YPR_RTE;
}
void mtxlf_rot_ypr_rte(const double * const  A, double *C)
{
	double (*a)[3] = (double (*)[3])A;
	MTX_ROT_YPR_RTE;
}
void mtxLf_rot_ypr_rte(const long double * const A, long double *C)
{
	long double (*a)[3] = (long double (*)[3])A;
	MTX_ROT_YPR_RTE;
}
void  mtxf_rot_ypr_rtq(const float * const A, float *C)
{
	float (*a)[3] = (float (*)[3])A;
	float rterm = 0.f;
	MTX_ROT_YPR_RTQ;
}
void mtxlf_rot_ypr_rtq(const double * const  A, double *C)
{
	double (*a)[3] = (double (*)[3])A;
	double rterm = 0.;
	MTX_ROT_YPR_RTQ;
}
void mtxLf_rot_ypr_rtq(const long double * const A, long double *C)
{
	long double (*a)[3] = (long double (*)[3])A;
	long double rterm = 0.L;
	MTX_ROT_YPR_RTQ;
}
void  mtxf_rot_ypr_qte(const float * const A, float *C)
{
	MTX_ROT_YPR_QTE;
}
void mtxlf_rot_ypr_qte(const double * const  A, double *C)
{
	MTX_ROT_YPR_QTE;
}
void mtxLf_rot_ypr_qte(const long double * const A, long double *C)
{
	MTX_ROT_YPR_QTE;
}
void  mtxf_rot_ypr_qtr(const float * const A,  float *C)
{
	float (*c)[3] = (float (*)[3])C;
	MTX_ROT_YPR_QTR;
}
void mtxlf_rot_ypr_qtr(const double * const  A, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	MTX_ROT_YPR_QTR;
}
void mtxLf_rot_ypr_qtr(const long double * const A, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	MTX_ROT_YPR_QTR;
}
void  mtxf_rot_ypr_wtp(const float * const A,  const float * const B, float *C)
{
	float sphi = sinf(A[0]);
	float cphi = cosf(A[0]);
	float cthe = cosf(A[1]);
	float tthe = tanf(A[1]);
	MTX_ROT_YPR_WTP;
}
void mtxlf_rot_ypr_wtp(const double * const  A, const double * const B,  double *C)
{
	float sphi = sin(A[0]);
	float cphi = cos(A[0]);
	float cthe = cos(A[1]);
	float tthe = tan(A[1]);
	MTX_ROT_YPR_WTP;
}
void mtxLf_rot_ypr_wtp(const long double * const A, const long double * const B, long double *C)
{
	float sphi = sinl(A[0]);
	float cphi = cosl(A[0]);
	float cthe = cosl(A[1]);
	float tthe = tanl(A[1]);
	MTX_ROT_YPR_WTP;
}
void  mtxf_rot_ypr_btr(const float * const A,  float *C)
{
	float (*c)[3] = (float (*)[3])C;
	float s_phi = sinf(A[0]);
	float c_phi = cosf(A[0]);
	float s_the = sinf(A[1]);
	float c_the = cosf(A[1]);
	float c_psi = cosf(A[2]);
	float s_psi = sinf(A[2]);
	MTX_ROT_YPR_BTR;
}
void mtxlf_rot_ypr_btr(const double * const  A, double *C)
{
	double (*c)[3] = (double (*)[3])C;
	double s_phi = sin(A[0]);
	double c_phi = cos(A[0]);
	double s_the = sin(A[1]);
	double c_the = cos(A[1]);
	double c_psi = cos(A[2]);
	double s_psi = sin(A[2]);
	MTX_ROT_YPR_BTR;
}
void mtxLf_rot_ypr_btr(const long double * const A, long double *C)
{
	long double (*c)[3] = (long double (*)[3])C;
	long double s_phi = sinl(A[0]);
	long double c_phi = cosl(A[0]);
	long double s_the = sinl(A[1]);
	long double c_the = cosl(A[1]);
	long double c_psi = cosl(A[2]);
	long double s_psi = sinl(A[2]);
	MTX_ROT_YPR_BTR;
}
void  mtxf_rot_ypr_btq(const float * const A,  float *C)
{
	float cphi2 = cosf(A[0]/2);
	float sphi2 = sinf(A[0]/2);
	float cthe2 = cosf(A[1]/2);
	float sthe2 = sinf(A[1]/2);
	float cpsi2 = cosf(A[2]/2);
	float spsi2 = sinf(A[2]/2);
	MTX_ROT_YPR_BTQ;
}
void mtxlf_rot_ypr_btq(const double * const  A, double *C)
{
	double cphi2 = cos(A[0]/2);
	double sphi2 = sin(A[0]/2);
	double cthe2 = cos(A[1]/2);
	double sthe2 = sin(A[1]/2);
	double cpsi2 = cos(A[2]/2);
	double spsi2 = sin(A[2]/2);
	MTX_ROT_YPR_BTQ;
}
void mtxLf_rot_ypr_btq(const long double * const A, long double *C)
{
	float cphi2 = cosl(A[0]/2);
	float sphi2 = sinl(A[0]/2);
	float cthe2 = cosl(A[1]/2);
	float sthe2 = sinl(A[1]/2);
	float cpsi2 = cosl(A[2]/2);
	float spsi2 = sinl(A[2]/2);
	MTX_ROT_YPR_BTQ;
}

void  mtxf_rot_ypr_bte(const float * const A, float *C)
{
	float (*a)[3] = (float (*)[3])A;
	MTX_ROT_YPR_BTE;
}
void mtxlf_rot_ypr_bte(const double * const  A, double *C)
{
	double (*a)[3] = (double (*)[3])A;
	MTX_ROT_YPR_BTE;
}
void mtxLf_rot_ypr_bte(const long double * const A, long double *C)
{
	long double (*a)[3] = (long double (*)[3])A;
	MTX_ROT_YPR_BTE;
}
/* FIXME
 * **** NOTES ****
 *
 * 1.Yaw-Pitch-Roll and Frame frd (front-right-down)
 *
 * -Take a body-fixed frd frame
 * -The yaw-pitch-roll sequence is defined in the frd frame as
 *
 *    PHI 	= [r; p; y]
 *    Cypr/ref = Croll(phi) * Cpitch(theta) * Cyaw(psi)
 *
 *    where:
 *    Croll  =[[	1			0			0		]
 *    	 [	0			cos(phi)		sin(phi)	]
 *    	 [	0			-sin(phi)		cos(phi)	]];
 * 
 *    Cpitch =[[	cos(theta)		0			-sin(theta)	]
 *    	 [	0			1			0		]
 *    	 [	sin(theta)		0			cos(theta)	]];
 * 
 *    Cyaw   =[[	cos(theta)		sin(theta)		0		]
 *    	 [	-sin(theta)		cos(theta)		0		]
 *    	 [	0			0			1		]];
 *
 * 
 * -Take a body-fixed XYZ frame with 'X' aligned fom center to front.
 * 	-Call it bXYZ (short for body-fixed XYZ frame)
 * 	-Rotate the bXYZ around X by 180° to get frd:
 * 		R_bXYZ/frd = R_X(180°)
 * 	or simply invert (*-1) the Y and Z axes and rotations.
 * 		theta(X) = theta(front)
 * 		theta(Y) = -theta(right)
 * 		theta(Z) = -theta(down)
 * 
 * 	-To get quantities expressed in bXYZ to body_frd (say, the mpu6050 gyro rates)
 * 		v_frd = C_bXYZ/frd * v_bXYZ
 * 
 * 	Example: the mpu6050 Gyro Rates^{bXYZ} are [P_bX, Q_bY, R_bZ]
 * 
 * -angular rate Omega^frd_b/r (body to reference, expressed in frd frame).
 * 			= [P;	  Q;	 R   ]
 * 			= [P_bX; -Q_bY; -R_bZ]  (related to mpu6050 gyros)
 * 
 * -ypr sequence rates PHI_dot = [r_dot; p_dot; y_dot] = H(PHI) * Omega^frd_b/r
 * 
 * 
 * -From PQR to PHI as dt->0 (assuming constant rotation rates)
 * 	Omega^frd_b/r 	= [P; Q; R]
 * 	H(PHI) 		= [[1		sin(phi)*tan(theta)	cos(phi)*tan(theta)	];
 * 			   [0		cos(phi)		-sin(phi)		];
 * 			   [0		sin(phi)/cos(phi)	cos(phi)/cos(theta)	]];
 *
 *	PHI_dot = H(PHI) * Omega^frd_b/r
 * 
 * 
 * -Integrate ypr rates to get final ypr:
 * 	YPR(t) = YPR(0) + int[0,t]{PHI_dot(t)dt} 
 * 	YPR(t) - YPR(0) = int[0,t]{PHI_dot(t)dt} 
 *
 * 	Assuming discrete time, contant rates, period T...
 * 	YPR_{k} = YPR_{k-1} + T * H(YPR_{k-1) * [P; Q; R]_{k}
 */ 
