
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "systemid_interface.h"

#include AIRCRAFT_UP1DIR

extern void get_system_id(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr){
	double aInitial = 3.6;			// initial amplitude, deg
	double aFinal	= 1.0;			// final amplitude, deg
	double fInitial = 0.5;			// initial frequency, Hz
	double fFinal	= 9.5667;			// final frequency, Hz 
	double tInitial = 7.0;			// start time, sec
	double duration = 17.0;			// chirp duration, sec
	double step_time = 9.53;		// step time, sec								
	
	
	switch(missionData_ptr -> sysid_select){
		case 0:
			//  apply to L3/R3	
			controlData_ptr->pitch_cmd_excite = step_chirp(time, tInitial, step_time, duration, fInitial, fFinal, aInitial, aFinal);
			controlData_ptr->l3    += controlData_ptr->pitch_cmd_excite;
			controlData_ptr->r3    += controlData_ptr->pitch_cmd_excite;
			controlData->cmp_status = time / (duration + tInitial) * 100.0;
			break;
		case 1:
			//  apply to L4/R4	
			controlData_ptr->pitch_cmd_excite = step_chirp(time, tInitial, step_time, duration, fInitial, fFinal, aInitial, aFinal);
			controlData_ptr->l4    += controlData_ptr->pitch_cmd_excite;
			controlData_ptr->r4    += controlData_ptr->pitch_cmd_excite;
			controlData->cmp_status = time / (duration + tInitial) * 100.0;
			break;
	}
}
