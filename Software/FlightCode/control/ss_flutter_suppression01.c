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

SS_A[0][0] = 0.919549501690361;
SS_A[1][0] = -0.125108682109459;
SS_A[2][0] = -0.409035147434411;
SS_A[3][0] = 0.205590885222226;
SS_A[4][0] = -0.015775875355845;
SS_A[5][0] = -0.001406680421075;
SS_A[0][1] = -0.047743880860428;
SS_A[1][1] = 0.821026745396760;
SS_A[2][1] = 0.035444966846579;
SS_A[3][1] = -0.229275087714509;
SS_A[4][1] = 0.023833879485393;
SS_A[5][1] = 0.002125184870832;
SS_A[0][2] = -0.044089185004068;
SS_A[1][2] = 0.007251236468321;
SS_A[2][2] = 0.841560373105004;
SS_A[3][2] = -0.260324996761026;
SS_A[4][2] = 0.000341841226675;
SS_A[5][2] = 0.000030480803748;
SS_A[0][3] = -0.128951184704373;
SS_A[1][3] = -0.050420819766708;
SS_A[2][3] = 0.080872941566268;
SS_A[3][3] = 0.607585891368810;
SS_A[4][3] = -0.171999834332040;
SS_A[5][3] = -0.015336632289849;
SS_A[0][4] = 0.029183719930931;
SS_A[1][4] = 0.019656693166176;
SS_A[2][4] = 0.027576640260991;
SS_A[3][4] = 0.071572964903077;
SS_A[4][4] = 0.156657133832944;
SS_A[5][4] = 0.102876393073599;
SS_A[0][5] = -0.024686856128342;
SS_A[1][5] = -0.016627830766634;
SS_A[2][5] = -0.023327408302894;
SS_A[3][5] = -0.060544423103801;
SS_A[4][5] = 0.059195782744200;
SS_A[5][5] = 0.912888460109760;
SS_B[0][0] = -0.009774475446401;
SS_B[1][0] = -0.005078000790923;
SS_B[2][0] = -0.014301904620701;
SS_B[3][0] = -0.023113505697113;
SS_B[4][0] = 0.002667873712402;
SS_B[5][0] = 0.000237885101935;
SS_B[0][1] = -0.000235586602129;
SS_B[1][1] = -0.000333248193362;
SS_B[2][1] = 0.000119494499797;
SS_B[3][1] = -0.000790885005448;
SS_B[4][1] = 0.000082256161969;
SS_B[5][1] = 0.000007334498400;
SS_B[0][2] = -0.000265554135578;
SS_B[1][2] = -0.000395498850450;
SS_B[2][2] = 0.000121423053504;
SS_B[3][2] = -0.000894546325391;
SS_B[4][2] = 0.000093241196442;
SS_B[5][2] = 0.000008313996055;
SS_C[0][0] = -0.044613140521524;
SS_C[0][1] = 0.067400647550181;
SS_C[0][2] = 0.000966704562358;
SS_C[0][3] = -0.486404247349172;
SS_C[0][4] = 3.262744623282229;
SS_C[0][5] = -2.759994520545006;
SS_D[0][0] = 0.007544571831379;
SS_D[0][1] = 0.000232615029587;
SS_D[0][2] = 0.000263679986398;

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
