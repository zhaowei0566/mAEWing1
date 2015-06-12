
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "systemid_interface.h"

#include AIRCRAFT_UP1DIR

extern void get_system_id( double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr){
	unsigned short sysid_select = missionData_ptr -> sysid_select; 	// mode switching
	double t0;
	
	if (time >= 10.0)
	{
		t0 = 10*floor(time/10.0);
	}
	else
	{
		t0 = 0;
	}
	// multisine pitch
	if(sysid_select == 2){

	}
	// 3-2-1-1 Roll on ganged then surface pairs
	else if(sysid_select == 1){

	}
	// 3-2-1-1 Pitch on ganged then surface pairs
	else{

	}
}

