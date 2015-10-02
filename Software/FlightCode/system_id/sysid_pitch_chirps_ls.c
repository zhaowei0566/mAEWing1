
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "systemid_interface.h"

#include AIRCRAFT_UP1DIR

extern void get_system_id(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr){
				
	switch(missionData_ptr -> sysid_select){
		case 0: 
			// chirp, open loop L3/R3, L4/R4		
			if(time < 15){
				controlData_ptr->pitch_cmd_excite = cos_chirp(time, 3.0, 12.0, 0.5, 10.0, 5.0, 3.0);
				controlData_ptr->l3    += controlData_ptr->pitch_cmd_excite;
				controlData_ptr->r3    += controlData_ptr->pitch_cmd_excite;
			}
			else{
				controlData_ptr->pitch_cmd_excite = cos_chirp(time, 18.0, 12.0, 0.5, 10.0, 5.0, 3.0);
				controlData_ptr->l4    += controlData_ptr->pitch_cmd_excite;
				controlData_ptr->r4    += controlData_ptr->pitch_cmd_excite;
			}
			
			controlData_ptr->cmp_status = time / (30.0) * 100.0;
			break;
		case 1:
			if(time < 11){
				// 3-2-1-1, open loop L3/R3
				controlData_ptr->pitch_cmd_excite = doublet3211(3.0, time, 1.0, 5.0*D2R);
				controlData_ptr->l3    += controlData_ptr->pitch_cmd_excite;
				controlData_ptr->r3    += controlData_ptr->pitch_cmd_excite;
			}
			if((time >= 11) && (time < 23)){
				// 3-2-1-1, open loop L3/R3
				controlData_ptr->pitch_cmd_excite = doublet3211(15.0, time, 0.8, 5.0*D2R);
				controlData_ptr->l3    += controlData_ptr->pitch_cmd_excite;
				controlData_ptr->r3    += controlData_ptr->pitch_cmd_excite;
			}
			if((time >= 23) && (time < 33)){
				// 3-2-1-1, open loop L3/R3
				controlData_ptr->pitch_cmd_excite = doublet3211(25.0, time, 0.6, 5.0*D2R);
				controlData_ptr->l3    += controlData_ptr->pitch_cmd_excite;
				controlData_ptr->r3    += controlData_ptr->pitch_cmd_excite;
			}
			
			controlData_ptr->cmp_status = time / (34.0) * 100.0;
			break;
	}
}
