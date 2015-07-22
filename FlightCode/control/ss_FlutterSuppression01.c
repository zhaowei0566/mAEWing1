/*! \file FlutterSuppression01.c
 *	\brief 
 *
 *	\details  none yet
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: ??? $
 */

#include "../globaldefs.h"
#include "ss_control_interface.h"
#include "../utils/matrix.h"

#define	 NCONT	1
#define	 NMEAS	3
#define	 NSTATE	6

static MATRIX SS_A, SS_B, SS_C, SS_D;
static MATRIX y, x, u;
static MATRIX tmpAx, tmpBy, tmpCx, tmpDy;
	
extern void init_ss_control(void){
	/*++++++++++++++++++++++++++++++++++++++++++++++++
	 *matrix creation for control computation
	 *++++++++++++++++++++++++++++++++++++++++++++++++*/	
	SS_A 			= mat_creat(NSTATE,NSTATE,ZERO_MATRIX);		// State Transition Matrix
	SS_B 			= mat_creat(NSTATE,NMEAS,ZERO_MATRIX);		// Input Matrix
	SS_C 			= mat_creat(NCONT,NSTATE,ZERO_MATRIX);		// Output Matrix
	SS_D 			= mat_creat(NCONT,NMEAS,ZERO_MATRIX);		// Feedthrough Matrix
	
	tmpAx 			= mat_creat(NSTATE,1,ZERO_MATRIX);		
	tmpBy 			= mat_creat(NSTATE,1,ZERO_MATRIX);		
	tmpCx 			= mat_creat(NCONT,1,ZERO_MATRIX);		
	tmpDy 			= mat_creat(NCONT,1,ZERO_MATRIX);		
	
	x				= mat_creat(NSTATE,1,ZERO_MATRIX);		// Controller State
	u				= mat_creat(NCONT,1,ZERO_MATRIX);		// Controller Output
	y				= mat_creat(NMEAS,1,ZERO_MATRIX);		// Controller Input

/*START ASSEMBLY OF MATRICES */ 

SS_A[0][0] = 0.846407946155538;
SS_A[1][0] = 0.091453849562772;
SS_A[2][0] = 0.127687920568237;
SS_A[3][0] = -0.189148424128363;
SS_A[4][0] = -0.081886585190010;
SS_A[5][0] = -0.007296247756822;
SS_A[0][1] = 0.139476298357592;
SS_A[1][1] = 0.671219709315664;
SS_A[2][1] = 0.125993590365211;
SS_A[3][1] = 0.031934814140849;
SS_A[4][1] = -0.295794171332990;
SS_A[5][1] = -0.026355813398025;
SS_A[0][2] = 0.116214221570980;
SS_A[1][2] = 0.001330681190889;
SS_A[2][2] = 0.706266645060421;
SS_A[3][2] = -0.438638209508636;
SS_A[4][2] = -0.004383915746387;
SS_A[5][2] = -0.000390615084955;
SS_A[0][3] = 0.303787040575470;
SS_A[1][3] = 0.613717785769810;
SS_A[2][3] = 0.157164588936770;
SS_A[3][3] = 0.317329971634928;
SS_A[4][3] = -0.117648044172085;
SS_A[5][3] = -0.010482660577349;
SS_A[0][4] = 0.010930716504714;
SS_A[1][4] = 0.043387096231538;
SS_A[2][4] = -0.007260947423485;
SS_A[3][4] = -0.033773175689348;
SS_A[4][4] = -0.159875781047440;
SS_A[5][4] = 0.074727269702155;
SS_A[0][5] = -0.013035873382662;
SS_A[1][5] = -0.051743057527087;
SS_A[2][5] = 0.008659340054231;
SS_A[3][5] = 0.040277583068490;
SS_A[4][5] = 0.035729725816772;
SS_A[5][5] = 0.910793761565147;
SS_B[0][0] = -0.012901323779007;
SS_B[1][0] = -0.036344000046417;
SS_B[2][0] = 0.011639640952689;
SS_B[3][0] = 0.029587224420522;
SS_B[4][0] = 0.006785224033613;
SS_B[5][0] = 0.000604576140523;
SS_B[0][1] = 0.000165804417848;
SS_B[1][1] = -0.001128953232325;
SS_B[2][1] = -0.000203765998221;
SS_B[3][1] = 0.000916036617314;
SS_B[4][1] = 0.000190526073007;
SS_B[5][1] = 0.000016976229129;
SS_B[0][2] = 0.000232669903065;
SS_B[1][2] = -0.001298473155091;
SS_B[2][2] = -0.000215757285480;
SS_B[3][2] = 0.001035064586589;
SS_B[4][2] = 0.000217635287828;
SS_B[5][2] = 0.000019391710827;
SS_C[0][0] = -0.120303087554086;
SS_C[0][1] = -0.434563878922168;
SS_C[0][2] = -0.006440598281678;
SS_C[0][3] = -0.172841777755902;
SS_C[0][4] = 1.232129386128730;
SS_C[0][5] = -1.469426332821276;
SS_D[0][0] = 0.009968463077264;
SS_D[0][1] = 0.000279910009547;
SS_D[0][2] = 0.000319737317483;

/*END ASSEMBLY OF MATRICES */  
}

// Main get_control function
extern void get_ss_control(double* measurement, double* control_signal) {
	
	y[0][0] = measurement[0];		// q
	y[1][0] = measurement[1];		// az
	y[2][0] = measurement[2];		// accel

	
	// Output Calculation
	tmpCx = mat_mul(SS_C,x,tmpCx);
	tmpDy = mat_mul(SS_D,y,tmpDy);
	u = mat_add(tmpCx,tmpDy,u);
	
	// State Update
	tmpAx = mat_mul(SS_A,x,tmpAx);
	tmpBy = mat_mul(SS_B,y,tmpBy);
	x = mat_add(tmpAx,tmpBy,x);
	
	control_signal[0]   = u[0][0];		// L4R4 symmetric [rad]
	
}

extern void close_ss_control(void){
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
extern void reset_ss_control(void){
 x = mat_scalMul(x,0.0,x);
}
