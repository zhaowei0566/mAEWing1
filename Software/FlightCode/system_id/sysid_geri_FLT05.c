
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "systemid_interface.h"

#include AIRCRAFT_UP1DIR

int overspeed = TRUE;
double overspeed_limit = 2.0; // overspeed threshold [m/s]

extern void get_system_id(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr){
	
	if (controlData_ptr->ias_cmd < (sensorData_ptr->adData_ptr->ias_filt - overspeed_limit)){
		overspeed = TRUE;
	}
	else{
		overspeed = FALSE;
	}
	
	switch(missionData_ptr -> sysid_select){
		case 0: 
			// SysID#1	
			controlData_ptr->surf4_excite = cos_chirp(time, 2.0, 20.0, 0.5, 18.5, 5.0, 5.0);
			if(overspeed == FALSE) {
				controlData_ptr->l4    += controlData_ptr->surf4_excite;
				controlData_ptr->r4    -= controlData_ptr->surf4_excite;
			}
			
			controlData_ptr->cmp_status = time / (23.0) * 100.0;
			break;
		case 1:	
			// SysID#2	
			controlData_ptr->surf3_excite = cos_chirp(time, 2.0, 20.0, 0.5, 18.5, 5.0, 5.0);
			if(overspeed == FALSE) {
				controlData_ptr->l3    += controlData_ptr->surf3_excite;
				controlData_ptr->r3    -= controlData_ptr->surf3_excite;
			}
			
			controlData_ptr->cmp_status = time / (23.0) * 100.0;
			break;
		case 2:
			// SysID#3	
			controlData_ptr->surf4_excite = cos_chirp(time, 2.0, 20.0, 0.5, 18.5, 5.0, 5.0);
			if(overspeed == FALSE) {
				controlData_ptr->l4    += controlData_ptr->surf4_excite;
				controlData_ptr->r4    -= controlData_ptr->surf4_excite;
			}
			
			controlData_ptr->cmp_status = time / (23.0) * 100.0;
			break;
	}
}
