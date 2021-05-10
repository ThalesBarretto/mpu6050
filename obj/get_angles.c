/*
 * Get current Yaw, Pitch, Roll.
 */
void mpu_get_angles(int *fd)
{

	/* Integrate the average rate between two readings
	 * Take the drift rate into account
	 */
	if (g_en == 1) {
		ypr_gyro[1] += sampling_time * ( (*Gz + *(Gz+1))/2 - *Gz_driftrate);
		ypr_gyro[2] += sampling_time * ( (*Gy + *(Gy+1))/2 - *Gy_driftrate);
		ypr_gyro[3] += sampling_time * ( (*Gx + *(Gx+1))/2 - *Gx_driftrate);

	}
	/*
	 * Integrate the last rate (not the current)
	 *
	ypr_gyro[1] += sampling_time * ( *(Gz+1) - *Gz_driftrate);
	ypr_gyro[2] += sampling_time * ( *(Gy+1) - *Gy_driftrate);
	ypr_gyro[3] += sampling_time * ( *(Gx+1) - *Gx_driftrate);
	 *
	 */

	/*
	 * Angle Conventions
	 * [1]YAW   = (Gz) CCW rotation about z axis
	 * [2]PITCH = (Gy) CCW rotation about y axis
	 * [3]ROLL  = (Gx) CCW rotation about x axis
	 *
	** Those are meaningless without specifyng the order
	 * in wich they are applied. We show 2 useful forms
	 * (thatis, forms with 3 equations and 2 variables):
	 *
	 * FORM1 - ROT_xyz =	[-sin(theta),
	 *		  	cos(theta)sin(phi),
	 *			cos(theta)cos(phi)]
	 * Solves to:
	 * Pitch(theta) = atan2(-Ax/sqrt(Ay2+Az2);
	 * Roll(phi) 	= -atan2(Ay/Az);
	 */
	if (a_en == 1) {
		ypr_accel[2] =  r2d*(atan2( -(*Ax), sqrt((*Ay2)+(*Az2)) ));
		ypr_accel[3] = -r2d*(atan2(  (*Ay), -(*Az) )) ;
	}
	/* FORM2 - ROT_yxz = 	[ -sin(theta)cos(phi),
	 *			sin(phi)
	 *			cos(theta)cos(phi)]
	 *
	 * Solves to:
	 * Pitch(theta)_yzx = atan2(-Ax/Az);  <----SIMPLER
	 * Roll(phi)_yzx = atan2(Ay/sqrt(Ax2+Az2);
	 */
	/* ROT_YXZ form code
	ypr_accel[2] = r2d*(atan2(data[1][0], data[3][0]));
	ypr_accel[3] = r2d*(atan2(data[2][0],srqt(squares[1]+squares[3]));
	*/

	/* Remeber, algthough the cannonical form is
	 *
	 * FORM3 - ROT_zyx =	[cos(theta)sin(phi)sin(psi) - cos(psi)sin(theta),
	 *			cos(psi)cos(theta)sin(phi) + sin(theta)sin(psi),
	 *			cos(theta)cos(phi)]
	 *
	 * but... it includes the "yaw" (psi) angle wich
	 * accelerometers only can't provide
	 */

	/*
	* TILT ANGLE(rho) formula
	* At rest, |Gp| = 1g, so:
	* Gp = [Ax,Ay,Az];
	* |Gp| = sqrt(Ax2+Ay2+Az2);
	* rho = acos(Az/|Gp|);
	rho = r2d*acos(-data[3][0]/A_magnitude);
	*/
	/*
	* -
	* v. 2.
	* At rest: |Gp| = 1g,
	*
	* rho = atan2( sqrt( Ax2 + Ay2 ), Az );
	*/
	if (a_en == 1) {
	rho = r2d*atan2( sqrt( (*Ax2) + (*Ay2) ), -(*Az) );
	//rho = r2d*atan2( sqrt( squares[1] + squares[2] ), data[3][0] );
	}

/*	GYRO INTEGRATION
	ypr_accel[1] = ts*(data[6][0] - calibration_driftrate[3]);
	ypr_accel[2] = -ts*((data[5][0]) - calibration_driftrate[2]);
	ypr_accel[3] = ts*((data[4][0]) - calibration_driftrate[1]);
*/

}
