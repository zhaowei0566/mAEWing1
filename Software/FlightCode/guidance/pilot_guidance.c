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
	double pitch_incp = sensorData_ptr->inceptorData_ptr->pitch;
	double roll_incp  = sensorData_ptr->inceptorData_ptr->roll;
	
	controlData_ptr->theta_cmd = (-20.5487*(pitch_incp-.0360)*(pitch_incp-.0360) + -44.2865*(pitch_incp-.0360))*D2R;
	controlData_ptr->phi_cmd = -1*(-4.4118*(roll_incp-0.0124)*(roll_incp-0.0124) + 72.5741*(roll_incp-0.0124))*D2R;
	controlData_ptr->ias_cmd = 23; // Speed command
}

