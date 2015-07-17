/*! \file FlutterSuppressionClosedLoop.c
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
SS_A[0][0] = -0.247097841212764;
SS_A[1][0] = 0.486728615338786;
SS_A[2][0] = -0.762773258135659;
SS_A[3][0] = -0.317142342159949;
SS_A[4][0] = -0.229491385192591;
SS_A[5][0] = -0.039090442946526;
SS_A[0][1] = 0.417026462004614;
SS_A[1][1] = 0.495930970913510;
SS_A[2][1] = -0.005334045473361;
SS_A[3][1] = -0.150315609824591;
SS_A[4][1] = -0.531102880461726;
SS_A[5][1] = -0.090465473595019;
SS_A[0][2] = 0.370698467229331;
SS_A[1][2] = -0.166684860157295;
SS_A[2][2] = 0.847068643695995;
SS_A[3][2] = -0.375327551186180;
SS_A[4][2] = 0.020864346691323;
SS_A[5][2] = 0.003553931025643;
SS_A[0][3] = 0.374753283161877;
SS_A[1][3] = -0.247504252323335;
SS_A[2][3] = 0.489382348181351;
SS_A[3][3] = 1.059587723360921;
SS_A[4][3] = 0.047116112180997;
SS_A[5][3] = 0.008025528686089;
SS_A[0][4] = 0.097587988715858;
SS_A[1][4] = 0.059188564635789;
SS_A[2][4] = 0.045226258679287;
SS_A[3][4] = 0.144866144171071;
SS_A[4][4] = -0.176798858156552;
SS_A[5][4] = 0.139972754745414;
SS_A[0][5] = -0.118139435616627;
SS_A[1][5] = -0.071653322432847;
SS_A[2][5] = -0.054750638328858;
SS_A[3][5] = -0.175374087913200;
SS_A[4][5] = 0.041072858978051;
SS_A[5][5] = 0.830375479219478;
SS_B[0][0] = 0.019397744559166;
SS_B[1][0] = -0.014806674137774;
SS_B[2][0] = 0.012328977818635;
SS_B[3][0] = 0.004211392000336;
SS_B[4][0] = 0.003096173006721;
SS_B[5][0] = 0.000527387004833;
SS_B[0][1] = 0.000688445123570;
SS_B[1][1] = 0.000503493829813;
SS_B[2][1] = 0.000231414551369;
SS_B[3][1] = 0.000911603944911;
SS_B[4][1] = -0.000229094343979;
SS_B[5][1] = -0.000039022812883;
SS_B[0][2] = 0.000028101530290;
SS_B[1][2] = -0.000853165302651;
SS_B[2][2] = 0.000054067203672;
SS_B[3][2] = -0.000955699375055;
SS_B[4][2] = 0.000278475633840;
SS_B[5][2] = 0.000047434180884;
SS_C[0][0] = -0.336400341472092;
SS_C[0][1] = -0.778518070271792;
SS_C[0][2] = 0.030584038462546;
SS_C[0][3] = 0.069065234031437;
SS_C[0][4] = 1.204562520756278;
SS_C[0][5] = -1.458236185002589;
SS_D[0][0] = 0.004538530524113;
SS_D[0][1] = -0.000335818337927;
SS_D[0][2] = 0.000408203986554;

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
