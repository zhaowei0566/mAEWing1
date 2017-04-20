/*! \file ss_flutter_supression.c
 *	\brief 
 *
 *	\details  none yet
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2017 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: ??? $
 */

#include "../globaldefs.h"
#include "../utils/matrix.h"
#include "ss_control_interface.h"

#define	 NCONT	1
#define	 NMEAS	1
#define	 NSTATE	4

static MATRIX SS_A, SS_B, SS_C, SS_D;
static MATRIX y, x, u;
static MATRIX tmpAx, tmpBy, tmpCx, tmpDy;
	
extern void init_ss03_control(void){
	/*++++++++++++++++++++++++++++++++++++++++++++++++
	 *matrix creation for control computation
	 *++++++++++++++++++++++++++++++++++++++++++++++++*/	
	SS_A 	= mat_creat(NSTATE, NSTATE, ZERO_MATRIX);	// State Transition Matrix
	SS_B 	= mat_creat(NSTATE, NMEAS, ZERO_MATRIX);	// Input Matrix
	SS_C 	= mat_creat(NCONT, NSTATE, ZERO_MATRIX);	// Output Matrix
	SS_D 	= mat_creat(NCONT, NMEAS, ZERO_MATRIX);		// Feedthrough Matrix
	
	tmpAx 	= mat_creat(NSTATE, 1, ZERO_MATRIX);	
	tmpBy 	= mat_creat(NSTATE, 1, ZERO_MATRIX);	
	tmpCx 	= mat_creat(NCONT, 1, ZERO_MATRIX);		
	tmpDy 	= mat_creat(NCONT, 1, ZERO_MATRIX);		
	
	x		= mat_creat(NSTATE, 1, ZERO_MATRIX);	// Controller State
	u		= mat_creat(NCONT, 1, ZERO_MATRIX);		// Controller Output
	y		= mat_creat(NMEAS, 1, ZERO_MATRIX);		// Controller Input

	/*START ASSEMBLY OF MATRICES */ 
	SS_A[0][0] = 0.6387705624237162;
	SS_A[1][0] = -0.3703204412285725;
	SS_A[2][0] = 0.0000000000000000;
	SS_A[3][0] = 0.0000000000000000;
	SS_A[0][1] = 0.3703204412285725;
	SS_A[1][1] = 0.6387705624237162;
	SS_A[2][1] = 0.0000000000000000;
	SS_A[3][1] = 0.0000000000000000;
	SS_A[0][2] = 0.1910102296774394;
	SS_A[1][2] = 0.6061822177178333;
	SS_A[2][2] = 0.2761795303892291;
	SS_A[3][2] = 0.0000000000000000;
	SS_A[0][3] = -0.0013218520093078;
	SS_A[1][3] = -0.0035808260372229;
	SS_A[2][3] = 0.0017989296253570;
	SS_A[3][3] = 0.2635971381157267;

	SS_B[0][0] = 0.0001315645961669;
	SS_B[1][0] = 0.0003349510263399;
	SS_B[2][0] = 0.0004101085822064;
	SS_B[3][0] = 0.1547691652166986;

	SS_C[0][0] = -1.0548576180066533;
	SS_C[0][1] = 23.5346076581946310;
	SS_C[0][2] = 0.0000000000000000;
	SS_C[0][3] = -0.0000000000000000;

	SS_D[0][0] = 0.0000000000000000;
	/*END ASSEMBLY OF MATRICES */  
}

// Main get_control function
extern void get_ss03_control(double* measurement, double* control_signal) {
	
	y[0][0] = measurement[0];		// accel_cr - accel_cf
	
	// Output Calculation
	tmpCx = mat_mul(SS_C, x, tmpCx);
	tmpDy = mat_mul(SS_D, y, tmpDy);
	u = mat_add(tmpCx, tmpDy, u);
	
	// State Update
	tmpAx = mat_mul(SS_A, x, tmpAx);
	tmpBy = mat_mul(SS_B, y, tmpBy);
	x = mat_add(tmpAx, tmpBy, x);
	
	control_signal[0]   = u[0][0];	// L1R1 symmetric [rad]
}

extern void close_ss03_control(void){
	//free memory space
	mat_free(SS_A);
	mat_free(SS_B);
	mat_free(SS_C);
	mat_free(SS_D);
	mat_free(tmpAx);
	mat_free(tmpBy);
	mat_free(tmpCx);
	mat_free(tmpDy);
	mat_free(x);
	mat_free(u);
	mat_free(y);
}


// Reset parameters to initial values
extern void reset_ss03_control(void){
	x = mat_scalMul(x, 0.0, x);
}
