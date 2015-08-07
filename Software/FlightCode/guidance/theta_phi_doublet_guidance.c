/*! \file doublet_phi_theta.c
 *	\brief Doublet commands on pitch and roll angle
 *
 *	\details
 *	\ingroup guidance_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: doublet_phi_theta.c 869 2012-08-06 22:40:38Z joh07594 $
 */

#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "../system_id/systemid_interface.h"
#include "guidance_interface.h"


extern void get_guidance(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr){
	unsigned short claw_mode 	= missionData_ptr -> claw_mode; 	// mode switching
	unsigned short claw_select 	= missionData_ptr -> claw_select; 	// mode switching
	static int t0_latched = FALSE;	// time latching
	static double t0 = 0;
	double exc_time;
	
	if((claw_mode == 0)&&(claw_select == 2)){
		if(t0_latched == FALSE){ // get the time since the theta command started
			t0 = time;
			t0_latched = TRUE;
		}
		exc_time = time - t0;
		controlData_ptr->theta_cmd = doublet(3, exc_time, 6, 4*D2R); // Pitch angle command
		controlData_ptr->phi_cmd = doublet(14, exc_time, 6, 15*D2R); // Roll angle command
	}
	else{
		t0_latched = FALSE;
	}
}

