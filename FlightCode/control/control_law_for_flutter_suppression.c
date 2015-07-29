#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "control_interface.h"
#include "ss_control_interface.h"
#include "../system_id/systemid_interface.h"

// ***********************************************************************************
#include AIRCRAFT_UP1DIR            // for Flight Code
// ***********************************************************************************

/// Definition of local functions: ****************************************************
static double roll_control (double phi_ref, double roll_angle, double rollrate, double delta_t, unsigned short gain_selector);
static double pitch_control(double the_ref, double pitch, double pitchrate, double delta_t, unsigned short gain_selector);
static double speed_control(double speed_ref, double airspeed, double delta_t);
static double zdot_control(double zdot_ref, double zdot, double pos_pitch_limit, double neg_pitch_limit, double delta_t);

// initialize pitch and roll angle tracking errors, integrators, and anti wind-up operators
// [0]: roll tracker, [1] theta tracker, [2] auto throttle, [3] zdot tracker
static double e[4] = {0,0,0,0}; 		 
static double integrator[4] = {0,0,0,0}; // integrator states for baseline
static short anti_windup[4]={1,1,1,1};   // integrates when anti_windup is 1

/// ****************************************************************************************

/// Gains
#ifdef AIRCRAFT_FENRIR
	static double roll_gain[3]  = {0.50,0.15,0.01};  	// PI gains for roll tracker and roll damper
	static double pitch_gain[3] = {-0.3,-0.40,-0.01};  	// PI gains for theta tracker and pitch damper
	static double v_gain[2] 	= {0.091, 0.020};		// PI gains for speed tracker
	static double zdot_gain[2]  = {-0.025,-0.05};		// PI gains for zdot tracker
#endif

#ifdef AIRCRAFT_SKOLL
	static double roll_gain[3]  = {0.5,0.15,0.01};  	// PI gains for roll tracker and roll damper when using Elevons
	static double pitch_gain[3] = {-0.3,-0.40,-0.01};  	// PI gains for pitch tracker and pitch damper when using Elevons
	static double roll_gain_single[3]  = {1.5,0.5,0.0};  		// PI gains for roll tracker and roll damper when using only Flap2
	static double pitch_gain_single[3] = {-0.75,-1.0,-0.0};  	// PI gains for pitch tracker and pitch damper when using only Flap3
	static double v_gain[2]     = {0.1, 0.020};			// PI gains for speed tracker
	static double zdot_gain[2]  = {-0.01,-0.05};		// PI gains for zdot tracker
#endif

#ifdef AIRCRAFT_HATI
	static double roll_gain[3]  = {0.50,0.15,0.01};  	// PI gains for roll tracker and roll damper
	static double pitch_gain[3] = {-0.3,-0.40,-0.01};  	// PI gains for theta tracker and pitch damper
	static double v_gain[2] 	= {0.091, 0.020};		// PI gains for speed tracker
	static double zdot_gain[2]  = {-0.025,-0.05};		// PI gains for zdot tracker
#endif

static double de;		// Delta elevator
static double da;		// Delta aileron
static double dthr;		// Delta throttle

static double ss_output[1] = {0}; 		// Delta L4R4 for FlutterSuppression
static double ss_input[3] = {0,0,0};	// Inputs for FlutterSuppression (q,az,acc)
/// *****************************************************************************************

extern void init_control(void) {
		init_ss_control();
};

extern void close_control(void) {
		close_ss_control();
};

extern void get_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr) {
/// Return control outputs based on references and feedback signals.
	unsigned short claw_mode 	= missionData_ptr -> claw_mode; 	// mode switching
	unsigned short claw_select 	= missionData_ptr -> claw_select; 	// mode switching
	unsigned short gain_selector = 0; 								// gain switching
	
	#ifdef AIRCRAFT_FENRIR
		double base_pitch_cmd= 0.0698;  	// (Trim value) 4 deg
		double base_acc= -9.81;  	// (Trim value) -1g
	#endif
	
	#ifdef AIRCRAFT_SKOLL
		double base_pitch_cmd= 0.0698;  	// (Trim value) 4 deg
		double base_acc= -9.81;  	// (Trim value) -1g
	#endif
	
	#ifdef AIRCRAFT_HATI
		double base_pitch_cmd= 0.0698;  	// (Trim value) 4 deg
		double base_acc= -9.81;  	// (Trim value) -1g
	#endif
	
	double phi   = navData_ptr->phi; 						// Bank angle
	double theta = navData_ptr->the - base_pitch_cmd; 		// Pitch angle - theta trim 
	double p     = sensorData_ptr->imuData_ptr->p; 			// Roll rate
	double q     = sensorData_ptr->imuData_ptr->q; 			// Pitch rate
	double ias   = sensorData_ptr->adData_ptr->ias_filt;	// Airspeed
	double zdot  = navData_ptr->vd; 						// Sink rate from nav filter
	
	double az 	 	= sensorData_ptr->imuData_ptr->az - base_acc; 			// IMU az measure - 1g
	double acc_lf   = sensorData_ptr->accelData_ptr->lf - base_acc; 	// left forward wing accelerometer measure - 1g
	double acc_rf   = sensorData_ptr->accelData_ptr->rf - base_acc;		// right forward wing accelerometer measure - 1g
	double acc_lr   = sensorData_ptr->accelData_ptr->lr - base_acc;		// left rear wing accelerometer measure - 1g
	double acc_rr   = sensorData_ptr->accelData_ptr->rr - base_acc;		// right rear wing accelerometer measure - 1g
	
	double phi_cmd;
    double theta_cmd;
	double ias_cmd;
	
	static double pos_pitch_limit;
	static double neg_pitch_limit;
	
	// Auxiliary  quantities to latch time and check gps
	static int t0_latched = FALSE;
	static double t0 = 0;
	static int t1_latched = FALSE;
	static double t1 = 0;
	double diff_time;
	static int flarenogps = FALSE;
	static double nogps_theta;
	
	// *** Z DOT GUIDANCE ***    # constant sinkrate approach -> wings level flare -> throttle cutoff
	if(claw_mode == 2){
		missionData_ptr -> run_excitation = 0;
		gain_selector = 0;
		
		// throttle cut 		
		if(claw_select == 2){
			controlData_ptr->zdot_cmd = 0.5;
			controlData_ptr->phi_cmd = 0;
			// if GPS is locked, use zdot otherwise just set theta and pray
			if(sensorData_ptr->gpsData_ptr->navValid == 0){
				controlData_ptr->theta_cmd = zdot_control(controlData_ptr->zdot_cmd, zdot, pos_pitch_limit, neg_pitch_limit, TIMESTEP);
			}
			else{
				controlData_ptr->theta_cmd = 1.0*D2R - base_pitch_cmd;
			}
			controlData_ptr->ias_cmd = -100;
		}
		// flare				# slow down with linear change in zdot_cmd and ias_cmd, adjust theta protection to positive values
		else if(claw_select == 1){
			if(t0_latched == FALSE){
				t0 = time;
				t0_latched = TRUE;
			}
			diff_time = time - t0;
			if(diff_time <=3){
				pos_pitch_limit = 20*D2R-diff_time*(12.0/3.0)*D2R-base_pitch_cmd;
				neg_pitch_limit = -20*D2R + diff_time*(20.0/3.0)*D2R-base_pitch_cmd;
				controlData_ptr->zdot_cmd = 3 - diff_time*(2.5/3.0);
				controlData_ptr->ias_cmd = 20 - diff_time*(3.0/3.0);
				nogps_theta = -5*D2R + diff_time*(6.0/3.0)*D2R-base_pitch_cmd;
			}
			else{
				pos_pitch_limit = 8*D2R-base_pitch_cmd;
				neg_pitch_limit = 0-base_pitch_cmd;
				controlData_ptr->zdot_cmd = 0.5;
				controlData_ptr->ias_cmd = 17;
				nogps_theta = 1.0*D2R - base_pitch_cmd;
			}
			controlData_ptr->phi_cmd = 0;
			// if GPS is locked, use zdot otherwise just set theta and pray
			if((sensorData_ptr->gpsData_ptr->navValid == 0)&&(flarenogps == FALSE)){
				controlData_ptr->theta_cmd = zdot_control(controlData_ptr->zdot_cmd, zdot, pos_pitch_limit, neg_pitch_limit, TIMESTEP);
			}
			else{
				controlData_ptr->theta_cmd = nogps_theta;
				flarenogps = TRUE;
			}
		}
		// approach 		# constant zdot_cmd and ias_cmd
		else{
			t0_latched = FALSE;
			pos_pitch_limit = 20*D2R-base_pitch_cmd;
			neg_pitch_limit = -20*D2R-base_pitch_cmd;
			controlData_ptr->zdot_cmd = 3;
			// if GPS is locked, use zdot otherwise just set theta and pray
			if(sensorData_ptr->gpsData_ptr->navValid == 0){
				controlData_ptr->theta_cmd = zdot_control(controlData_ptr->zdot_cmd, zdot, pos_pitch_limit, neg_pitch_limit, TIMESTEP);
			}
			else{
				controlData_ptr->theta_cmd = -5*D2R - base_pitch_cmd;
				// reset the z dot states
				e[3] = 0;
				integrator[3] = 0;
				anti_windup[3] = 1;
			}
			controlData_ptr->ias_cmd = 20;	
		}
	}
	// *** EXPERIMENTS MODE ***    # perform different experiments, selected with claw_select 
	// use with TRES_SYSID.c
	else if(claw_mode == 0){  // FLUTTER CONTROLLER WITH EXCITATION AT DIFFERENT SPEEDS
	
		gain_selector = 1;
		ss_input[0] = q; 		
		ss_input[1] = az; 		
		ss_input[2] = 0.25*(acc_lf+acc_lr+acc_rf+acc_rr);   			
		
		get_ss_control(ss_input, ss_output);
		
		missionData_ptr -> run_excitation = 1;
			
		if(claw_select == 2){ 		
			controlData_ptr->ias_cmd = 30;   					// SPEED SETTING
			missionData_ptr -> sysid_select = 0;
		}
		else if(claw_select == 1){ 	// Chirp Flaps 3, then 4
			controlData_ptr->ias_cmd = 25; 						// SPEED SETTING
			missionData_ptr -> sysid_select = 0;
		}
		else{ 						// Chirp Flaps 1, then 2
			controlData_ptr->ias_cmd = 20; 						// SPEED SETTING
			missionData_ptr -> sysid_select = 0;
		}
		
		// reset the z dot states
		e[3] = 0;
		integrator[3] = 0;
		anti_windup[3] = 1;
	}
	// *** REGULAR PILOT MODE ***    # Pilot flies through Fly-by-Attitude control law with active Autothrottle
	else{
		gain_selector = 0;
		missionData_ptr -> run_excitation = 0;
		
		// reset the z dot states
		e[3] = 0;
		integrator[3] = 0;
		anti_windup[3] = 1;
	}
	
	
	// reset flutter suppression
	if (claw_mode != 0)
	{
		ss_output[0] = ss_input[0] = ss_input[1] = ss_input[2] = 0;
		reset_ss_control();
	}
	
	

	// assign the reference commands
	phi_cmd 	= controlData_ptr->phi_cmd;
	theta_cmd 	= controlData_ptr->theta_cmd;
	ias_cmd 	= controlData_ptr->ias_cmd;

	// assign the control commands
	controlData_ptr->dthr 		= speed_control(ias_cmd, ias, TIMESTEP); 						// Throttle [nd]
	controlData_ptr->l3  	  	= pitch_control(theta_cmd, theta, q, TIMESTEP, gain_selector); 	// use elevator L3+R3 [rad]
	controlData_ptr->r3  		= controlData_ptr->l3; 											//
	controlData_ptr->l2  		= roll_control(phi_cmd, phi, p, TIMESTEP, gain_selector);		// use aileron L2-R2 [rad]
	controlData_ptr->r2  		= -controlData_ptr->l2;											//  
	controlData_ptr->l1   		= 0;															// L1 [rad]
	controlData_ptr->r1   		= 0; 															// R1 [rad]

	// elevon mixing 
	if(claw_mode == 0){  // no elevon mixing for experiments
		controlData_ptr->l4   		= ss_output[0]; 											// L4 [rad]
		controlData_ptr->r4   		= ss_output[0]; 											// R4 [rad]
	}
	else{  // elevon mixing for regular operation and landing
		controlData_ptr->l4   		= controlData_ptr->l3 + controlData_ptr->l2; 				// L4 [rad]
		controlData_ptr->r4   		= controlData_ptr->r3 + controlData_ptr->r2; 				// R4 [rad]
	}
}

// Roll get_control law: angles in radians. Rates in rad/s. Time in seconds
static double roll_control (double phi_ref, double roll_angle, double rollrate, double delta_t, unsigned short gain_selector)
{
	// ROLL tracking controller implemented here

	// roll attitude tracker
	e[0] = phi_ref - roll_angle;
	integrator[0] += e[0]*delta_t*anti_windup[0]; //roll error integral (rad)

	//proportional term + integral term              - roll damper term
	if (gain_selector==0)
	{
	da  = roll_gain[0]*e[0] + roll_gain[1]*integrator[0] - roll_gain[2]*rollrate;
	}
	else if (gain_selector==1)
	{
	da  = roll_gain_single[0]*e[0] + roll_gain_single[1]*integrator[0] - roll_gain_single[2]*rollrate;
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
	// pitch attitude tracker
	e[1] = the_ref - pitch;
	integrator[1] += e[1]*delta_t*anti_windup[1]; //pitch error integral

    // proportional term + integral term               - pitch damper term
	if (gain_selector==0)
	{
    de = pitch_gain[0]*e[1] + pitch_gain[1]*integrator[1] - pitch_gain[2]*pitchrate;    // Elevator output
	}
	else if (gain_selector==1)
	{
    de = pitch_gain_single[0]*e[1] + pitch_gain_single[1]*integrator[1] - pitch_gain_single[2]*pitchrate;    // Elevator output
	}

	//eliminate wind-up
	if      (de >= L3_MAX-PITCH_SURF_TRIM && e[1] < 0) {anti_windup[1] = 0; de = L3_MAX-PITCH_SURF_TRIM;}  //stop integrating
	else if (de >= L3_MAX-PITCH_SURF_TRIM && e[1] > 0) {anti_windup[1] = 1; de = L3_MAX-PITCH_SURF_TRIM;}
	else if (de <= L3_MIN-PITCH_SURF_TRIM && e[1] < 0) {anti_windup[1] = 1; de = L3_MIN-PITCH_SURF_TRIM;}
	else if (de <= L3_MIN-PITCH_SURF_TRIM && e[1] > 0) {anti_windup[1] = 0; de = L3_MIN-PITCH_SURF_TRIM;}  //stop integrating
	else {anti_windup[1] = 1;}

	return de;  //rad
}

// Airspeed get_control law: Speed in m/s. Time in seconds
static double speed_control(double speed_ref, double airspeed, double delta_t)
{
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

// Sink rate get_control law: sink rate in m/s. Pitch limits in rad. Time in seconds
static double zdot_control(double zdot_ref, double zdot, double pos_pitch_limit, double neg_pitch_limit, double delta_t)
{
	double theta_ref;
	
	// pitch attitude tracker
	e[3] = zdot_ref - zdot;
	integrator[3] += e[3]*delta_t*anti_windup[3]; //zdot error integral

    	// proportional term + integral term
    	theta_ref = zdot_gain[0]*e[3] + zdot_gain[1]*integrator[3];    // theta_ref output
	
	//eliminate wind-up on theta ref integral
	if      (theta_ref >= pos_pitch_limit && e[3] < 0) {anti_windup[3] = 0; theta_ref = pos_pitch_limit;}  //stop integrating
	else if (theta_ref >= pos_pitch_limit && e[3] > 0) {anti_windup[3] = 1; theta_ref = pos_pitch_limit;}
	else if (theta_ref <= neg_pitch_limit && e[3] < 0) {anti_windup[3] = 1; theta_ref = neg_pitch_limit;}
	else if (theta_ref <= neg_pitch_limit && e[3] > 0) {anti_windup[3] = 0; theta_ref = neg_pitch_limit;}  //stop integrating
	else {anti_windup[3] = 1;}
	
	return theta_ref;
}

// Reset parameters to initial values
extern void reset_control(struct control *controlData_ptr){

	integrator[0] = integrator[1] = integrator[2] = integrator[3] = 0;
	anti_windup[0] = anti_windup[1] = anti_windup[2] = anti_windup[3] = 1;
	e[0] = e[1] = e[2] = e[3] = 0;

	controlData_ptr->dthr = 0; 	// throttle
	controlData_ptr->l1   = 0;	// L1 [rad]
    controlData_ptr->r1   = 0; 	// R1 [rad]
	controlData_ptr->l2   = 0;	// L2 [rad]
    controlData_ptr->r2   = 0; 	// R2 [rad]
	controlData_ptr->l3   = 0;	// L3 [rad]
    controlData_ptr->r3   = 0; 	// R3 [rad]	
	controlData_ptr->l4   = 0; 	// L4 [rad]
	controlData_ptr->r4   = 0; 	// R4 [rad]
	
	ss_output[0] = ss_input[0] = ss_input[1] = ss_input[2] = 0;
	reset_ss_control();
}
