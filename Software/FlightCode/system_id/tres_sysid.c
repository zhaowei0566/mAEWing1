
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "systemid_interface.h"

#include AIRCRAFT_UP1DIR

extern void get_system_id( double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr){
	double t0;

	// reset time if greater than 16 seconds	
	if (time >= 19.0)
	{
		t0 = 19*floor(time/19.0);
	}
	else
	{
		t0 = 0;
	}

	// 3-2-1-1 Pitch on ganged then surface pairs
	if((time-t0)<10.5){
		// 3-2-1-1 symmetric excitations at 2 Hz to inboard flaps
		controlData_ptr->pitch_cmd_excite = doublet3211(3+t0, time, 0.175, 7*D2R);	// total duration is 7x duration input
		controlData_ptr->l3 = controlData_ptr->l3 + controlData_ptr->pitch_cmd_excite;
		controlData_ptr->r3 = controlData_ptr->r3 + controlData_ptr->pitch_cmd_excite;		
	}
	else{
		// 3-2-1-1 symmetric excitations at 2 Hz to outboard flaps
		controlData_ptr->pitch_cmd_excite = doublet3211(11+t0, time, 0.175, 7*D2R); 	// total duration is 7x duration input
		controlData_ptr->l4 = controlData_ptr->l4 + controlData_ptr->pitch_cmd_excite;		
		controlData_ptr->r4 = controlData_ptr->r4 + controlData_ptr->pitch_cmd_excite;
	}
}
