
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "systemid_interface.h"

#include AIRCRAFT_UP1DIR

extern void get_system_id( double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr){
	unsigned short sysid_select = missionData_ptr -> sysid_select; 	// mode switching
	double t0;

	// reset time if greater than 23 seconds	
	if (time >= 23.0)
	{
		t0 = 23*floor(time/23.0);
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
		if((time-t0)<4.5){
			// 3-2-1-1 antisymmetric excitations at 2 Hz to inboard flaps
			controlData_ptr->roll_cmd_excite = doublet3211(0+t0, time, 0.175, 7*D2R);	// total duration is 7x duration input
			controlData_ptr->l1     = controlData_ptr->l1 +  controlData_ptr->roll_cmd_excite;
			controlData_ptr->r1   = controlData_ptr->r1 -  controlData_ptr->roll_cmd_excite;			
		}
		else if(((time-t0)>=4.5)&&((time-t0)<11.5)){
			// 3-2-1-1 antisymmetric excitations at 2 Hz to ganged roll surfaces
			controlData_ptr->roll_cmd_excite = doublet3211(5+t0, time, 0.175, 3.5*D2R); 	// total duration is 7x duration input
			controlData_ptr->l2 = controlData_ptr->l2 + controlData_ptr->roll_cmd_excite;
			controlData_ptr->r2 = controlData_ptr->r2 - controlData_ptr->roll_cmd_excite;
			controlData_ptr->l4 = controlData_ptr->l4 + controlData_ptr->roll_cmd_excite;		
			controlData_ptr->r4 = controlData_ptr->r4 - controlData_ptr->roll_cmd_excite;			
		}
		else if(((time-t0)>=11.5)&&((time-t0)<16.5)){
			// 3-2-1-1 antisymmetric excitations at 2 Hz to ailerons
			controlData_ptr->roll_cmd_excite = doublet3211(12+t0, time, 0.175, 7*D2R); 	// total duration is 7x duration input
			controlData_ptr->l2 = controlData_ptr->l2 + controlData_ptr->roll_cmd_excite;
			controlData_ptr->r2 = controlData_ptr->r2 - controlData_ptr->roll_cmd_excite;			
		}
		else{
			// 3-2-1-1 antisymmetric excitations at 2 Hz to outboard flaps
			controlData_ptr->roll_cmd_excite = doublet3211(17+t0, time, 0.175, 7*D2R); 	// total duration is 7x duration input
			controlData_ptr->l4 = controlData_ptr->l4 + controlData_ptr->roll_cmd_excite;		
			controlData_ptr->r4 = controlData_ptr->r4 - controlData_ptr->roll_cmd_excite;		
		}	
	}
	// 3-2-1-1 Pitch on ganged then surface pairs
	else{
		if((time-t0)<4.5){
			// 3-2-1-1 symmetric excitations at 2 Hz to inboard flaps
			controlData_ptr->pitch_cmd_excite = doublet3211(0+t0, time, 0.175, 7*D2R);	// total duration is 7x duration input
			controlData_ptr->l1     = controlData_ptr->l1 +  controlData_ptr->pitch_cmd_excite;
			controlData_ptr->r1   = controlData_ptr->r1 +  controlData_ptr->pitch_cmd_excite;			
		}
		else if(((time-t0)>=4.5)&&((time-t0)<11.5)){
			// 3-2-1-1 symmetric excitations at 2 Hz to ganged pitch surfaces
			controlData_ptr->pitch_cmd_excite = doublet3211(5+t0, time, 0.175, 3.5*D2R); 	// total duration is 7x duration input
			controlData_ptr->l3 = controlData_ptr->l3 + controlData_ptr->pitch_cmd_excite;
			controlData_ptr->r3 = controlData_ptr->r3 + controlData_ptr->pitch_cmd_excite;
			controlData_ptr->l4 = controlData_ptr->l4 + controlData_ptr->pitch_cmd_excite;		
			controlData_ptr->r4 = controlData_ptr->r4 + controlData_ptr->pitch_cmd_excite;			
		}
		else if(((time-t0)>=11.5)&&((time-t0)<16.5)){
			// 3-2-1-1 symmetric excitations at 2 Hz to elevators
			controlData_ptr->pitch_cmd_excite = doublet3211(12+t0, time, 0.175, 7*D2R); 	// total duration is 7x duration input
			controlData_ptr->l3 = controlData_ptr->l3 + controlData_ptr->pitch_cmd_excite;
			controlData_ptr->r3 = controlData_ptr->r3 + controlData_ptr->pitch_cmd_excite;
		}
		else{
			// 3-2-1-1 symmetric excitations at 2 Hz to outboard flaps
			controlData_ptr->pitch_cmd_excite = doublet3211(17+t0, time, 0.175, 7*D2R); 	// total duration is 7x duration input
			controlData_ptr->l4 = controlData_ptr->l4 + controlData_ptr->pitch_cmd_excite;		
			controlData_ptr->r4 = controlData_ptr->r4 + controlData_ptr->pitch_cmd_excite;
		}
	}
}

