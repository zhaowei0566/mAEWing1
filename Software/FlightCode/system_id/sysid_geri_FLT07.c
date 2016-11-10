
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "systemid_interface.h"

#include AIRCRAFT_UP1DIR

int overspeed = TRUE;
double overspeed_limit = 2.0; // overspeed threshold [m/s]
int indx;

extern void get_system_id(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr){
	
	if (controlData_ptr->ias_cmd < (sensorData_ptr->adData_ptr->ias_filt - overspeed_limit)){
		overspeed = TRUE;
	}
	else{
		overspeed = FALSE;
	}
	
	indx = (int) (time*150.0);
	
	switch(missionData_ptr -> sysid_select){
		case 0: 
			// SysID#1
			
			if (time < 1.5){
			}
			else if((time >= 1.5) && (time < 21.5)){
			controlData_ptr->surf4_excite = cos_chirp(time, 0, 20.0, 0.8, 18.5, 5.0, 5.0);
			
				if(overspeed == FALSE) {
				controlData_ptr->l4    += controlData_ptr->surf4_excite;
				controlData_ptr->r4    -= controlData_ptr->surf4_excite;
				}
			}
			
			controlData_ptr->cmp_status = time / (21.5) * 100.0;
			break;
		case 1:
			// SysID#2
			
			if (time < 1.5){
			}
			else if((time >= 1.5) && (time < 13.5)){
				controlData_ptr->surf1_excite = playback_OMS2_1(indx - 1.5*150, 5.0*D2R);
				if(overspeed == FALSE) {
					controlData_ptr->l1    += controlData_ptr->surf1_excite;
					controlData_ptr->r1    += controlData_ptr->surf1_excite;
				}
			}
			else if((time >= 13.5) && (time < 15.0)){
			}
			else if((time >= 15.0) && (time < 27.0)){
				controlData_ptr->surf2_excite = playback_OMS2_1(indx - 15.0*150, 5.0*D2R);
				controlData_ptr->surf4_excite = playback_OMS2_2(indx - 15.0*150, 3.2*D2R);
				if(overspeed == FALSE) {
					controlData_ptr->l2    += controlData_ptr->surf2_excite;
					controlData_ptr->r2    -= controlData_ptr->surf2_excite;
					controlData_ptr->l4    += controlData_ptr->surf4_excite;
					controlData_ptr->r4    -= controlData_ptr->surf4_excite;
				}
			}
			controlData_ptr->cmp_status = time / (27.0) * 100.0;
			break;
		case 2:
			// SysID#3
			
			if (time < 1.5){
			}
			else if((time >= 1.5) && (time < 21.5)){
			controlData_ptr->surf4_excite = cos_chirp(time, 0, 20.0, 0.8, 18.5, 5.0, 5.0);
			
				if(overspeed == FALSE) {
				controlData_ptr->l4    += controlData_ptr->surf4_excite;
				controlData_ptr->r4    -= controlData_ptr->surf4_excite;
				}
			}
			
			controlData_ptr->cmp_status = time / (21.5) * 100.0;
			break;
	}
}
