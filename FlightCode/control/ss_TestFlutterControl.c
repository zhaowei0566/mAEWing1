#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "control_interface.h"
#include "ss_control_interface.h"

// ***********************************************************************************
#include AIRCRAFT_UP1DIR            // for Flight Code
// ***********************************************************************************

/// Definition of local functions: ****************************************************
static double roll_control (double phi_ref, double roll_angle, double rollrate, double delta_t);
static double pitch_control(double the_ref, double pitch, double pitchrate, double delta_t);
static double speed_control(double speed_ref, double airspeed, double delta_t);
static double zdot_control(double zdot_ref, double zdot, double pos_pitch_limit, double neg_pitch_limit, double delta_t);

// initialize pitch and roll angle tracking errors, integrators, and anti wind-up operators
// 0 values correspond to the roll tracker, 1 values correspond to the theta tracker
static double e[4] = {0,0,0,0};
static double integrator[4] = {0,0,0,0};
static short anti_windup[4]={1,1,1,1};   // integrates when anti_windup is 1


	static double roll_gain[3]  = {0.5,0.15,0.0};  	// PI gains for roll tracker and roll damper
	static double pitch_gain[3] = {-0.3,-0.4,-0.0};  	// PI gains for pitch tracker and pitch damper
	static double v_gain[2]     = {0.1, 0.020};			// PI gains for speed tracker
	static double zdot_gain[2]  = {-0.01,-0.05};		// PI gains for zdot tracker
	
/*  ACTUAL GAINS ?
	static double roll_gain[3]  = {1.5,0.5,0.0};  	// PI gains for roll tracker and roll damper
	static double pitch_gain[3] = {-0.75,-1.0,-0.0};  	// PI gains for pitch tracker and pitch damper
	static double v_gain[2]     = {0.1, 0.020};			// PI gains for speed tracker
	static double zdot_gain[2]  = {-0.01,-0.05};		// PI gains for zdot tracker
*/

	
/// ****************************************************************************************


static double da; // Delta aileron
static double de; // Delta elevator
static double dthr;

static double ss_output[1] = {0};
static double ss_input[3] = {0,0,0};

/// *****************************************************************************************

extern void init_control(void){
	init_ss_control();
};

extern void close_control(void){
	close_ss_control();
};


extern void get_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr) {
/// Return control outputs based on references and feedback signals.
	unsigned short claw_mode = missionData_ptr -> claw_mode; 		// mode switching
	unsigned short claw_select = missionData_ptr -> claw_select; 	// mode switching
	
		double base_pitch_cmd= 0.0698;  	// (Trim value) 4 deg
		double base_acc= -9.81;  	// (Trim value) -1g

	double phi   = navData_ptr->phi;
	double theta = navData_ptr->the - base_pitch_cmd; //subtract theta trim value to convert to delta coordinates
	double p     = sensorData_ptr->imuData_ptr->p; // Roll rate
	double q     = sensorData_ptr->imuData_ptr->q; // Pitch rate
	double ias   = sensorData_ptr->adData_ptr->ias_filt;
	double zdot  = navData_ptr->vd;
	
	// for flutter suppression
	double az 	 = sensorData_ptr->imuData_ptr->az - base_acc;
	double acc_lf   = sensorData_ptr->accelData_ptr->lf - base_acc; 
	double acc_rf   = sensorData_ptr->accelData_ptr->rf - base_acc;
	double acc_lr   = sensorData_ptr->accelData_ptr->lr - base_acc;
	double acc_rr   = sensorData_ptr->accelData_ptr->rr - base_acc;

	
	
	double 	phi_cmd 	= controlData_ptr->phi_cmd;
    double 	theta_cmd 	= controlData_ptr->theta_cmd;
	double 	ias_cmd 	= controlData_ptr->ias_cmd;
	
	
	static double pos_pitch_limit;
	static double neg_pitch_limit;
	static int t0_latched = FALSE;
	static double t0 = 0;
	double diff_time;
	
	
	
// *** STATE SPACE CONTROLLER PART
if (claw_mode==0) 
{
	ss_input[0] = -q; 		
	ss_input[1] = -az; 		
	ss_input[2] = -0.25*(acc_lf+acc_lr+acc_rf+acc_rr);   			
		
	get_ss_control(ss_input, ss_output);
}
else 
{
	ss_output[0] = 0;
	reset_ss_control();
}

controlData_ptr->l4 = controlData_ptr->r4 = ss_output[0]; // L4 R4 [rad]

//  	**********************************  

// *** BASELINE CONTROLLER PART	
	controlData_ptr->dthr = speed_control(ias_cmd, ias, TIMESTEP);
	controlData_ptr->de   = pitch_control(theta_cmd, theta, q, TIMESTEP);
	controlData_ptr->da   = roll_control(phi_cmd, phi, p, TIMESTEP);   
	
	controlData_ptr->l1   = 0;			// L1 [rad]
	controlData_ptr->r1   = 0; 			// R1 [rad]
//  	**********************************
}	
	
	

// Roll get_control law: angles in radians. Rates in rad/s. Time in seconds
static double roll_control(double phi_ref, double roll_angle, double rollrate, double delta_t)
{
	// ROLL tracking controller implemented here

	// roll attitude tracker
	e[0] = phi_ref - roll_angle;
	integrator[0] += e[0]*delta_t*anti_windup[0]; //roll error integral (rad)

	//proportional term + integral term              - roll damper term
	da  = roll_gain[0]*e[0] + roll_gain[1]*integrator[0] - roll_gain[2]*rollrate;

	//eliminate windup
	if      (da >= AILERON_MAX-ROLL_SURF_TRIM && e[0] < 0) {anti_windup[0] = 1; da = AILERON_MAX-ROLL_SURF_TRIM;}
	else if (da >= AILERON_MAX-ROLL_SURF_TRIM && e[0] > 0) {anti_windup[0] = 0; da = AILERON_MAX-ROLL_SURF_TRIM;}  //stop integrating
	else if (da <= AILERON_MIN-ROLL_SURF_TRIM && e[0] < 0) {anti_windup[0] = 0; da = AILERON_MIN-ROLL_SURF_TRIM;}  //stop integrating
	else if (da <= AILERON_MIN-ROLL_SURF_TRIM && e[0] > 0) {anti_windup[0] = 1; da = AILERON_MIN-ROLL_SURF_TRIM;}
	else {anti_windup[0] = 1;}

    return da;
}

// Pitch get_control law: angles in radians. Rates in rad/s. Time in seconds
/*                                                  __________
                   _______________                 |          |  theta
theta_cmd  _      |               |       _   de   |          |---------
    ----->|+|---->| Theta Tracker |----->|+|------>| Aircraft |  q      |
           -      |_______________|       -        |          |-----    |
           ^            (PI)              ^        |__________|     |   |
         - |                            - |        ______________   |   |
           |                              |       |              |  |   |
           |                               -------| Pitch Damper |<-    |
           |                                      |______________|      |
            ------------------------------------------------------------     */
static double pitch_control(double the_ref, double pitch, double pitchrate, double delta_t)
{
	// pitch attitude tracker
	e[1] = the_ref - pitch;
	integrator[1] += e[1]*delta_t*anti_windup[1]; //pitch error integral

    		// proportional term + integral term               - pitch damper term
    		de = pitch_gain[0]*e[1] + pitch_gain[1]*integrator[1] - pitch_gain[2]*pitchrate;    // Elevator output

	//eliminate wind-up
	if      (de >= ELEVATOR_MAX-PITCH_SURF_TRIM && e[1] < 0) {anti_windup[1] = 0; de = ELEVATOR_MAX-PITCH_SURF_TRIM;}  //stop integrating
	else if (de >= ELEVATOR_MAX-PITCH_SURF_TRIM && e[1] > 0) {anti_windup[1] = 1; de = ELEVATOR_MAX-PITCH_SURF_TRIM;}
	else if (de <= ELEVATOR_MIN-PITCH_SURF_TRIM && e[1] < 0) {anti_windup[1] = 1; de = ELEVATOR_MIN-PITCH_SURF_TRIM;}
	else if (de <= ELEVATOR_MIN-PITCH_SURF_TRIM && e[1] > 0) {anti_windup[1] = 0; de = ELEVATOR_MIN-PITCH_SURF_TRIM;}  //stop integrating
	else {anti_windup[1] = 1;}

	return de;  //rad
}

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
	controlData_ptr->de   = 0; 	// elevator
	controlData_ptr->da   = 0; 	// rudder
	controlData_ptr->l1   = 0;	// L1 [rad]
    controlData_ptr->r1   = 0; 	// R1 [rad]
	controlData_ptr->l4   = 0; 	// L4 [rad]
	controlData_ptr->r4   = 0; 	// R4 [rad]

	ss_output[0] = 0;
	reset_ss_control();  // reset the state space controller state
}
