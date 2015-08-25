#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "control_interface.h"
#include "../system_id/systemid_interface.h"

// ***********************************************************************************
#include AIRCRAFT_UP1DIR            // for Flight Code
// ***********************************************************************************

/// Definition of local functions: ****************************************************
static double pilot_theta(struct sensordata *sensorData_ptr);
static double pilot_phi(struct sensordata *sensorData_ptr);

static double roll_control (double phi_ref, double roll_angle, double rollrate, double delta_t, unsigned short gain_selector);
static double pitch_control(double the_ref, double pitch, double pitchrate, double delta_t, unsigned short gain_selector);
static double speed_control(double speed_ref, double airspeed, double delta_t);

void approach_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr);
void flare_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr);
void pilot_flying(double time, double ias_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr);
void pilot_flying_inner(double time, double ias_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr);
void open_loop(double time, double ias_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr);
void reset_tracker();

// initialize pitch and roll angle tracking errors, integrators, and anti wind-up operators
// 0 values correspond to the roll tracker, 1 values correspond to the theta tracker
static double e[3] = {0,0,0};
static double integrator[3] = {0,0,0};
static short anti_windup[3]={1,1,1};   // integrates when anti_windup is 1

/// ****************************************************************************************

/// Gains
#ifdef AIRCRAFT_SKOLL
	static double roll_gain[3]  		= {0.5,0.15,0.01};  	// PI gains for roll tracker and roll damper
	static double roll_gain_single[3]  	= {1.5,0.5,0.0};  		// PI gains for roll tracker and roll damper when using only Flap2
	static double pitch_gain[3] 		= {-0.3,-0.40,-0.00};  	// PI gains for pitch tracker and pitch damper
	static double pitch_gain_single[3] 	= {-0.75,-1.0,-0.0};  	// PI gains for pitch tracker and pitch damper when using only Flap3
	static double v_gain[2]     		= {0.1, 0.020};			// PI gains for speed tracker
#endif

double base_pitch_cmd		= 0.0698;  				// Trim value 4 deg
double trim_speed			= 23;					// Trim airspeed, m/s
double approach_theta 		= -6.5*D2R;				// Absolute angle for the initial approach
double approach_speed 		= 20;					// Approach airspeed, m/s
double flare_theta 			= 1*D2R;				// Absolute angle for the flare
double flare_speed 			= 17;					// Flare airspeed, m/s
double pilot_flare_delta	= 1;					// Delta flare airspeed if the pilot is landing, m/s
double exp_speed			= 27;					// Speed to run the experiments at, m/s

/// *****************************************************************************************

extern void init_control(void) {
};

extern void close_control(void) {
};

extern void get_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr) {

	unsigned short claw_mode 	= missionData_ptr -> claw_mode; 	// mode switching
	unsigned short claw_select 	= missionData_ptr -> claw_select; 	// mode switching
	static int t0_latched = FALSE;	// time latching
	static double t0 = 0;
	static int t2_latched = FALSE;	// time latching
	static double t2 = 0;
	double flare_time;
	double speed_cmd;

	switch(claw_mode){
		case 0: // experiment mode
			t0_latched = FALSE;
			switch(claw_select){
				case 0: // chirp, open loop L3/R3 and L4/R4 at 23 m/s
					t2_latched = FALSE;
					reset_tracker();
					missionData_ptr -> run_excitation = 1;
					missionData_ptr -> sysid_select = 0;
					open_loop(time, trim_speed, sensorData_ptr, navData_ptr, controlData_ptr);
					break;
				case 1: // open loop flying at 35 m/s, ramp from 23 to 35 over 20 seconds
					reset_tracker();
					missionData_ptr -> run_excitation = 0;
					if(t2_latched == FALSE){ // get the time since starting
						t2 = time;
						t2_latched = TRUE;
					}
					
					if((time - t2) <= 20.0){ // ramp commands
						speed_cmd = 23.0 + (time - t2)*((35.0 - 23.0)/20.0);
					}
					else{ // final values
						speed_cmd = 35.0;
					}
					
					open_loop(time, speed_cmd, sensorData_ptr, navData_ptr, controlData_ptr);
					
					controlData_ptr->cmp_status = speed_cmd;
					break;
				default: // open loop flying at 23 m/s
					t2_latched = FALSE;
					missionData_ptr -> run_excitation = 0;
					open_loop(time, trim_speed, sensorData_ptr, navData_ptr, controlData_ptr);
					break;
			}
			break;
		case 2: // autolanding mode
			t2_latched = FALSE;
			missionData_ptr -> run_excitation = 0;
			switch(claw_select){
				case 0: // approach
					t0_latched = FALSE;
					approach_control(time, sensorData_ptr, navData_ptr, controlData_ptr);
					break;
				case 1: // flare
					if(t0_latched == FALSE){ // get the time since flare started
						t0 = time;
						t0_latched = TRUE;
					}
					flare_time = time - t0;
					flare_control(flare_time, sensorData_ptr, navData_ptr, controlData_ptr);
					break;
				default: // pilot guidance
					t0_latched = FALSE;
					pilot_flying(time, (flare_speed + pilot_flare_delta), sensorData_ptr, navData_ptr, controlData_ptr);
					break;
			}
			break;
		default: // pilot mode
			t2_latched = FALSE;
			missionData_ptr -> run_excitation = 0;
			t0_latched = FALSE;
			pilot_flying(time, trim_speed,sensorData_ptr, navData_ptr, controlData_ptr);
			break;
	}
}

static double pilot_theta(struct sensordata *sensorData_ptr){
	double pitch_incp = sensorData_ptr->inceptorData_ptr->pitch;
	double theta_cmd;
	
	theta_cmd = 20*D2R*pitch_incp;
	return theta_cmd;
}

static double pilot_phi(struct sensordata *sensorData_ptr){
	double roll_incp  = sensorData_ptr->inceptorData_ptr->roll;
	double phi_cmd;
	
	phi_cmd = 35*D2R*roll_incp;
	return phi_cmd;
}

void open_loop(double time, double ias_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr){
	double ias   = sensorData_ptr->adData_ptr->ias_filt;	// filtered airspeed
	double roll_incp  = sensorData_ptr->inceptorData_ptr->roll;
	double pitch_incp = sensorData_ptr->inceptorData_ptr->pitch;

	controlData_ptr->ias_cmd = ias_cmd;
	
	controlData_ptr->dthr 	= speed_control(ias_cmd, ias, TIMESTEP);			// Throttle [ND 0-1]
	controlData_ptr->l1   	= 0;												// L1 [rad]
    controlData_ptr->l2   	= roll_incp*L2_MAX;									// L2 [rad]
	controlData_ptr->l3   	= -1*pitch_incp*(L3_MAX-15*D2R);					// L3 [rad]
	controlData_ptr->l4   	= 0; 												// L4 [rad]
    controlData_ptr->r1   	= 0; 												// R1 [rad]
	controlData_ptr->r2   	= -1*controlData_ptr->l2; 							// R2 [rad]
	controlData_ptr->r3   	= controlData_ptr->l3;								// R3 [rad]
	controlData_ptr->r4   	= 0; 												// R4 [rad]
}

void pilot_flying(double time, double ias_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr){
	double phi   = navData_ptr->phi;						// roll angle
	double theta = navData_ptr->the - base_pitch_cmd; 		// subtract theta trim value to convert to delta coordinates
	double p     = sensorData_ptr->imuData_ptr->p; 			// roll rate
	double q     = sensorData_ptr->imuData_ptr->q; 			// pitch rate
	double ias   = sensorData_ptr->adData_ptr->ias_filt;	// filtered airspeed
	double phi_cmd, theta_cmd;
	
	controlData_ptr->phi_cmd	= pilot_phi(sensorData_ptr);		// phi from the pilot stick
	controlData_ptr->theta_cmd	= pilot_theta(sensorData_ptr);		// theta from the pilot stick
	controlData_ptr->ias_cmd = ias_cmd;
	
	theta_cmd 	= controlData_ptr->theta_cmd;
	phi_cmd		= controlData_ptr->phi_cmd;
	
	controlData_ptr->dthr 	= speed_control(ias_cmd, ias, TIMESTEP);			// Throttle [ND 0-1]
	controlData_ptr->l1   	= 0;												// L1 [rad]
    controlData_ptr->l2   	= roll_control(phi_cmd, phi, p, TIMESTEP, 0);		// L2 [rad]
	controlData_ptr->l3   	= pitch_control(theta_cmd, theta, q, TIMESTEP, 0);	// L3 [rad]
	controlData_ptr->l4   	= controlData_ptr->l3 + controlData_ptr->l2; 		// L4 [rad]
    controlData_ptr->r1   	= 0; 												// R1 [rad]
	controlData_ptr->r2   	= -1*controlData_ptr->l2; 							// R2 [rad]
	controlData_ptr->r3   	= controlData_ptr->l3;								// R3 [rad]
	controlData_ptr->r4   	= controlData_ptr->r3 + controlData_ptr->r2; 		// R4 [rad]
}

void pilot_flying_inner(double time, double ias_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr){
	double phi   = navData_ptr->phi;						// roll angle
	double theta = navData_ptr->the - base_pitch_cmd; 		// subtract theta trim value to convert to delta coordinates
	double p     = sensorData_ptr->imuData_ptr->p; 			// roll rate
	double q     = sensorData_ptr->imuData_ptr->q; 			// pitch rate
	double ias   = sensorData_ptr->adData_ptr->ias_filt;	// filtered airspeed
	double phi_cmd, theta_cmd;
	
	controlData_ptr->phi_cmd	= pilot_phi(sensorData_ptr) + controlData_ptr->phi_cmd;		// phi from the pilot stick + guidance
	controlData_ptr->theta_cmd	= pilot_theta(sensorData_ptr) + controlData_ptr->theta_cmd;	// theta from the pilot stick + guidance
	controlData_ptr->ias_cmd = ias_cmd;
	
	theta_cmd 	= controlData_ptr->theta_cmd;
	phi_cmd		= controlData_ptr->phi_cmd;
	
	controlData_ptr->dthr 	= speed_control(ias_cmd, ias, TIMESTEP);			// Throttle [ND 0-1]
	controlData_ptr->l1   	= 0;												// L1 [rad]
    controlData_ptr->l2   	= roll_control(phi_cmd, phi, p, TIMESTEP, 1);		// L2 [rad]
	controlData_ptr->l3   	= pitch_control(theta_cmd, theta, q, TIMESTEP, 1);	// L3 [rad]
	controlData_ptr->l4   	= 0; 												// L4 [rad]
    controlData_ptr->r1   	= 0; 												// R1 [rad]
	controlData_ptr->r2   	= -1*controlData_ptr->l2; 							// R2 [rad]
	controlData_ptr->r3   	= controlData_ptr->l3;								// R3 [rad]
	controlData_ptr->r4   	= 0; 												// R4 [rad]
}

void flare_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr){
	double ramp_time = 6.0;
	double min_speed = 8;
	double cut_time  = 5;
	double phi   = navData_ptr->phi;						// roll angle
	double theta = navData_ptr->the - base_pitch_cmd; 		// subtract theta trim value to convert to delta coordinates
	double p     = sensorData_ptr->imuData_ptr->p; 			// roll rate
	double q     = sensorData_ptr->imuData_ptr->q; 			// pitch rate
	double ias   = sensorData_ptr->adData_ptr->ias_filt;	// filtered airspeed
	static int t1_latched = FALSE;	// time latching
	static double t1 = 0;
	double ias_cmd, phi_cmd, theta_cmd;
	
	if(time <= ramp_time){ // ramp commands
		ias_cmd = approach_speed - time*((approach_speed - flare_speed)/ramp_time);
		theta_cmd = approach_theta + time*((flare_theta - approach_theta)/ramp_time);
	}
	else{ // final values
		ias_cmd = flare_speed;
		theta_cmd = flare_theta;
	}
	
	if(ias < min_speed){ // engine cutoff
		if(t1_latched == FALSE){
			t1 = time;
			t1_latched = TRUE;
		}
		if((time-t1) > cut_time){
			ias_cmd = -100;	
		}
	}
	else{
		t1_latched = FALSE;
	}
				
	controlData_ptr->theta_cmd 	= theta_cmd - base_pitch_cmd;	// delta theta command [rad]
	controlData_ptr->ias_cmd 	= ias_cmd;						// absolute airspeed command [m/s]
	controlData_ptr->phi_cmd	= 0;							// wings level
	
	theta_cmd 	= controlData_ptr->theta_cmd;
	ias_cmd		= controlData_ptr->ias_cmd;
	phi_cmd		= controlData_ptr->phi_cmd;
	
	controlData_ptr->dthr 	= speed_control(ias_cmd, ias, TIMESTEP);			// Throttle [ND 0-1]
	controlData_ptr->l1   	= 0;												// L1 [rad]
    controlData_ptr->l2   	= roll_control(phi_cmd, phi, p, TIMESTEP, 0);		// L2 [rad]
	controlData_ptr->l3   	= pitch_control(theta_cmd, theta, q, TIMESTEP, 0);	// L3 [rad]
	controlData_ptr->l4   	= controlData_ptr->l3 + controlData_ptr->l2; 		// L4 [rad]
    controlData_ptr->r1   	= 0; 												// R1 [rad]
	controlData_ptr->r2   	= -1*controlData_ptr->l2; 							// R2 [rad]
	controlData_ptr->r3   	= controlData_ptr->l3;								// R3 [rad]
	controlData_ptr->r4   	= controlData_ptr->r3 + controlData_ptr->r2; 		// R4 [rad]
}

// approach control, 
void approach_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr){

	double phi   = navData_ptr->phi;						// roll angle
	double theta = navData_ptr->the - base_pitch_cmd; 		// subtract theta trim value to convert to delta coordinates
	double p     = sensorData_ptr->imuData_ptr->p; 			// roll rate
	double q     = sensorData_ptr->imuData_ptr->q; 			// pitch rate
	double ias   = sensorData_ptr->adData_ptr->ias_filt;	// filtered airspeed
	double ias_cmd, phi_cmd, theta_cmd;
	
	controlData_ptr->theta_cmd 	= approach_theta - base_pitch_cmd;	// delta theta command [rad]
	controlData_ptr->ias_cmd 	= approach_speed;					// absolute airspeed command [m/s]
	controlData_ptr->phi_cmd	= pilot_phi(sensorData_ptr);		// phi from the pilot stick
	
	theta_cmd 	= controlData_ptr->theta_cmd;
	ias_cmd		= controlData_ptr->ias_cmd;
	phi_cmd		= controlData_ptr->phi_cmd;
	
	controlData_ptr->dthr 	= speed_control(ias_cmd, ias, TIMESTEP);			// Throttle [ND 0-1]
	controlData_ptr->l1   	= 0;												// L1 [rad]
    controlData_ptr->l2   	= roll_control(phi_cmd, phi, p, TIMESTEP, 0);		// L2 [rad]
	controlData_ptr->l3   	= pitch_control(theta_cmd, theta, q, TIMESTEP, 0);	// L3 [rad]
	controlData_ptr->l4   	= controlData_ptr->l3 + controlData_ptr->l2; 		// L4 [rad]
    controlData_ptr->r1   	= 0; 												// R1 [rad]
	controlData_ptr->r2   	= -1*controlData_ptr->l2; 							// R2 [rad]
	controlData_ptr->r3   	= controlData_ptr->l3;								// R3 [rad]
	controlData_ptr->r4   	= controlData_ptr->r3 + controlData_ptr->r2; 		// R4 [rad]
}

// Roll get_control law: angles in radians. Rates in rad/s. Time in seconds
static double roll_control (double phi_ref, double roll_angle, double rollrate, double delta_t, unsigned short gain_selector)
{
	double da;

	// roll attitude tracker
	e[0] = phi_ref - roll_angle;
	integrator[0] += e[0]*delta_t*anti_windup[0]; //roll error integral (rad)

	//proportional term + integral term              - roll damper term
	if(gain_selector == 1){
		da  = roll_gain_single[0]*e[0] + roll_gain_single[1]*integrator[0] - roll_gain_single[2]*rollrate;
	}
	else{
		da  = roll_gain[0]*e[0] + roll_gain[1]*integrator[0] - roll_gain[2]*rollrate;
	}
	//eliminate windup
	if      (da >= L2_MAX-ROLL_SURF_TRIM && e[0] < 0) {anti_windup[0] = 1; da = L2_MAX-ROLL_SURF_TRIM;}
	else if (da >= L2_MAX-ROLL_SURF_TRIM && e[0] > 0) {anti_windup[0] = 0; da = L2_MAX-ROLL_SURF_TRIM;}  //stop integrating
	else if (da <= L2_MIN-ROLL_SURF_TRIM && e[0] < 0) {anti_windup[0] = 0; da = L2_MIN-ROLL_SURF_TRIM;}  //stop integrating
	else if (da <= L2_MIN-ROLL_SURF_TRIM && e[0] > 0) {anti_windup[0] = 1; da = L2_MIN-ROLL_SURF_TRIM;}
	else {anti_windup[0] = 1;}

    return da;
}

// Pitch get_control law: angles in radians. Rates in rad/s. Time in seconds
static double pitch_control(double the_ref, double pitch, double pitchrate, double delta_t, unsigned short gain_selector)
{
	double de;
	
	// pitch attitude tracker
	e[1] = the_ref - pitch;
	integrator[1] += e[1]*delta_t*anti_windup[1]; //pitch error integral

    // proportional term + integral term               - pitch damper term
	if(gain_selector == 1){
		de = pitch_gain_single[0]*e[1] + pitch_gain_single[1]*integrator[1] - pitch_gain_single[2]*pitchrate;    // Elevator output
	}
	else{
		de = pitch_gain[0]*e[1] + pitch_gain[1]*integrator[1] - pitch_gain[2]*pitchrate;    // Elevator output
	}
		
	//eliminate wind-up
	if      (de >= L3_MAX-PITCH_SURF_TRIM && e[1] < 0) {anti_windup[1] = 0; de = L3_MAX-PITCH_SURF_TRIM;}  //stop integrating
	else if (de >= L3_MAX-PITCH_SURF_TRIM && e[1] > 0) {anti_windup[1] = 1; de = L3_MAX-PITCH_SURF_TRIM;}
	else if (de <= L3_MIN-PITCH_SURF_TRIM && e[1] < 0) {anti_windup[1] = 1; de = L3_MIN-PITCH_SURF_TRIM;}
	else if (de <= L3_MIN-PITCH_SURF_TRIM && e[1] > 0) {anti_windup[1] = 0; de = L3_MIN-PITCH_SURF_TRIM;}  //stop integrating
	else {anti_windup[1] = 1;}

	return de;  //rad
}

static double speed_control(double speed_ref, double airspeed, double delta_t)
{
	double dthr;
	
	// Speed tracker
	e[2] = speed_ref - airspeed;
	integrator[2] += e[2]*delta_t*anti_windup[2]; // altitude error integral

	// proportional term  + integral term
	dthr = v_gain[0]*e[2] + v_gain[1]*integrator[2];    // Throttle output

	//eliminate wind-up on airspeed integral
	if      (dthr >= THROTTLE_MAX-THROTTLE_TRIM && e[2] < 0) {anti_windup[2] = 1; dthr = THROTTLE_MAX-THROTTLE_TRIM;}
	else if (dthr >= THROTTLE_MAX-THROTTLE_TRIM && e[2] > 0) {anti_windup[2] = 0; dthr = THROTTLE_MAX-THROTTLE_TRIM;}  //stop integrating
	else if (dthr <= THROTTLE_MIN-THROTTLE_TRIM && e[2] < 0) {anti_windup[2] = 0; dthr = THROTTLE_MIN-THROTTLE_TRIM;}  //stop integrating
	else if (dthr <= THROTTLE_MIN-THROTTLE_TRIM && e[2] > 0) {anti_windup[2] = 1; dthr = THROTTLE_MIN-THROTTLE_TRIM;}
	else {anti_windup[2] = 1;}

	return dthr; // non dimensional
}

void reset_tracker(){
	integrator[0] = integrator[1] = integrator[2] = 0;
	anti_windup[0] = anti_windup[1] = anti_windup[2] = 1;
	e[0] = e[1] = e[2] = 0;
}

// Reset parameters to initial values
extern void reset_control(struct control *controlData_ptr){

	integrator[0] = integrator[1] = integrator[2] = 0;
	anti_windup[0] = anti_windup[1] = anti_windup[2] = 1;
	e[0] = e[1] = e[2] = 0;

	controlData_ptr->dthr = 0; 	// throttle
	controlData_ptr->l1   = 0;	// L1 [rad]
	controlData_ptr->l2   = 0;	// L2 [rad]
	controlData_ptr->l3   = 0;	// L3 [rad]
	controlData_ptr->l4   = 0; 	// L4 [rad]
	controlData_ptr->r1   = 0; 	// R1 [rad]
	controlData_ptr->r2   = 0; 	// R2 [rad]
	controlData_ptr->r3   = 0; 	// R3 [rad]
	controlData_ptr->r4   = 0; 	// R4 [rad]
}
