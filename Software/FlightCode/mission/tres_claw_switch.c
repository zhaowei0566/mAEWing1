
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "mission_interface.h"

extern void run_mission(struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct mission *missionData_ptr) {
	double pilot_mode = sensorData_ptr->inceptorData_ptr->yaw;
	double pilot_select = sensorData_ptr->inceptorData_ptr->throttle;
	
	if((pilot_mode > -0.2)&&(pilot_mode < 0.2)){
		missionData_ptr -> claw_mode = 1;
	}
	else if(pilot_mode < -0.4){
		missionData_ptr -> claw_mode = 2;
	}
	else{
		missionData_ptr -> claw_mode = 0;
	}
	
	if((pilot_select > 0.4)&&(pilot_select < 0.6)){
		missionData_ptr -> claw_select = 1;
	}
	else if(pilot_select > 0.6){
		missionData_ptr -> claw_select = 2;
	}
	else{
		missionData_ptr -> claw_select = 0;
	}
}
