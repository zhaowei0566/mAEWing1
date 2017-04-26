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

#define	 NCONT	2
#define	 NMEAS	3

static MATRIX M;
static MATRIX y, u;
	
extern void init_ss02_control(void){
	/*++++++++++++++++++++++++++++++++++++++++++++++++
	 *matrix creation for control computation
	 *++++++++++++++++++++++++++++++++++++++++++++++++*/	
	M 	= mat_creat(NCONT, NMEAS, ZERO_MATRIX);		// M Matrix
	
	u	= mat_creat(NCONT, 1, ZERO_MATRIX);		// Controller Output
	y	= mat_creat(NMEAS, 1, ZERO_MATRIX);		// Controller Input

	/*START ASSEMBLY OF MATRICES */
	M[0][0] = -0.0106791738845144;
	M[1][0] = 0.0001403242257218;
	M[0][1] = 0.0064966837270341;
	M[1][1] = -0.0000853663517060;
	M[0][2] = -0.0003611883202100;
	M[1][2] = 0.0000047460104987;
	/*END ASSEMBLY OF MATRICES */  
}

// Main get_control function
extern void get_ss02_control(double* measurement, double* control_signal) {
	
	y[0][0] = measurement[0];		// nzCBfwd
	y[1][0] = measurement[1];		// nzCBaft
	y[2][0] = measurement[2];		// nzLRWTave
	
	// Output Calculation
	u = mat_mul(M, y, u);
	
	control_signal[0]   = u[0][0];	// L1R1 symmetric [rad]
	control_signal[1]   = u[1][0];	// L4R4 symmetric [rad]
}

extern void close_ss02_control(void){
	//free memory space
	mat_free(M);
	mat_free(u);
	mat_free(y);
}


// Reset parameters to initial values
extern void reset_ss02_control(void){
}
