
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "systemid_interface.h"

#include AIRCRAFT_UP1DIR

extern void get_system_id( double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr){
	unsigned short claw_select = missionData_ptr -> claw_select; 	// mode switching
	double t0;
	
	if (time >= 10.0)
	{
		t0 = 10*floor(time/10.0);
	}
	else
	{
		t0 = 0;
	}
	
	if(claw_select == 2){

	}

	else if(claw_select == 1){

	}

	else{

	}
}

