
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "systemid_interface.h"

#include AIRCRAFT_UP1DIR

extern void get_system_id(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr){
	double aInitial = 5.0;			// initial amplitude, deg
	double aFinal	= 1.5;			// final amplitude, deg
	double fInitial = 0.5;			// initial frequency, Hz
	double fFinal	= 9.7308;		// final frequency, Hz 
	double tInitial = 1.0;			// start time, sec
	double duration = 15.0;			// chirp duration, sec
	double step_time = 10.97;		// step time, sec		
	double time_too;				// local time for the second excitation
	
	
	switch(missionData_ptr -> sysid_select){
		case 0:
		
			if(time < (duration + tInitial)){
				//  apply to L3/R3	
				controlData_ptr->pitch_cmd_excite = step_chirp(time, tInitial, step_time, duration, fInitial, fFinal, aInitial, aFinal);
				controlData_ptr->l3    += controlData_ptr->pitch_cmd_excite;
				controlData_ptr->r3    += controlData_ptr->pitch_cmd_excite;
			}
			else{	
				time_too = time - (duration + tInitial);
				
				//  apply to L4/R4	
				controlData_ptr->pitch_cmd_excite = step_chirp(time_too, tInitial+2.0, step_time, duration, fInitial, fFinal, aInitial, aFinal);
				controlData_ptr->l4    += controlData_ptr->pitch_cmd_excite;
				controlData_ptr->r4    += controlData_ptr->pitch_cmd_excite;
			}
			
			controlData->cmp_status = time / (2*(duration + tInitial)+2) * 100.0;
			
			break;
	}
}
