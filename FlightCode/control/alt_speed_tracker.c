
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "control_interface.h"

//////////////////////////////////////////////////////////////
#include AIRCRAFT_UP1DIR
//////////////////////////////////////////////////////////////

/// Definition of local functions: ****************************************************
static double roll_control (double phi_ref, double roll_angle, double rollrate, double delta_t);
static double altitude_control(double alt_ref, double altitude, double pitch, double pitchrate, double delta_t);
static double speed_control(double speed_ref, double airspeed, double delta_t);

// initialize pitch and roll angle tracking errors, integrators, and anti wind-up operators
// 0 values correspond to the roll tracker, 1 values correspond to the theta tracker, 2 values to altitude tracker, 3 to speed tracker
static double e[4] = {0,0,0,0};
static double integrator[4] = {0,0,0,0};
static short anti_windup[4]={1,1,1,1};   // integrates when anti_windup is 1

/// Roll, Pitch, Altitude, & Speed Controller Gains
#ifdef AIRCRAFT_FENRIR
	static double roll_gain[3]  = {-0.64,-0.20,-0.07};  // PI gains for roll tracker and roll damper
	static double pitch_gain[3] = {-0.90,-0.30,-0.08};  // PI gains for theta tracker and pitch damper
	static double alt_gain[2] 	= {0.023,0.0010}; 		// PI gains for altitude tracker
	static double v_gain[2] 	= {0.091, 0.020};		// PI gains for speed tracker
#endif

static double da; // Delta aileron
static double de; // Delta elevator
static double dthr; // Delta throttle

/// Return control outputs based on references and feedback signals.
extern void get_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr) {

#ifdef AIRCRAFT_FENRIR
	double base_pitch_cmd= 0.1222;  	// (Trim value) 7 deg
#endif

	double phi   = navData_ptr->phi;					    // Roll angle
	double theta = navData_ptr->the - base_pitch_cmd; 	    // Pitch angle: subtract theta trim value to convert to delta coordinates
	double p     = sensorData_ptr->imuData_ptr->p; 		    // Roll rate
	double q     = sensorData_ptr->imuData_ptr->q;		    // Pitch rate

    // Snapshots
	if(time <= 50*TIMESTEP){
		controlData_ptr->init_alt = sensorData_ptr->adData_ptr->h_filt;   	// store initial ALTITUDE
	}

	controlData_ptr->dthr = speed_control(controlData_ptr->ias_cmd, sensorData_ptr->adData_ptr->ias_filt, TIMESTEP);
    controlData_ptr->de   = altitude_control(controlData_ptr->h_cmd, sensorData_ptr->adData_ptr->h_filt - controlData_ptr->init_alt, theta, q, TIMESTEP);
    controlData_ptr->da   = roll_control(controlData_ptr->phi_cmd, phi, p, TIMESTEP);
	controlData_ptr->l1   = 0;		// L1 [rad]
    controlData_ptr->r1   = 0; 		// R1 [rad]
	controlData_ptr->l4   = 0; 		// L4 [rad]
	controlData_ptr->r4   = 0; 		// R4 [rad]
}

static double altitude_control(double alt_ref, double altitude, double pitch, double pitchrate, double delta_t)
{
	double pitch_limit = 0.349066; // Pitch angle saturation limit (20 degrees)
	double h_out;

	// Altitude tracker
	e[2] = alt_ref - altitude;
	integrator[2] += e[2]*delta_t*anti_windup[2]; // altitude error integral

	//	proportional term + integral term   (output of altitude PI block)
	h_out = alt_gain[0]*e[2] + alt_gain[1]*integrator[2];

	if	(abs(h_out) >= pitch_limit){
		h_out = sign(h_out)*pitch_limit;
	}

	// pitch attitude tracker
	e[1] = h_out - pitch;
	integrator[1] += e[1]*delta_t*anti_windup[1]; //pitch error integral

    //proportional term + integral term               - pitch damper term
    de = pitch_gain[0]*e[1] + pitch_gain[1]*integrator[1] - pitch_gain[2]*pitchrate;    // Elevator output

	//eliminate wind-up on theta integral
	if      (de >= ELEVATOR_MAX-PITCH_SURF_TRIM && e[1] < 0) {anti_windup[1] = 0; de = ELEVATOR_MAX-PITCH_SURF_TRIM;}  //stop integrating
	else if (de >= ELEVATOR_MAX-PITCH_SURF_TRIM && e[1] > 0) {anti_windup[1] = 1; de = ELEVATOR_MAX-PITCH_SURF_TRIM;}
	else if (de <= -ELEVATOR_MAX-PITCH_SURF_TRIM && e[1] < 0) {anti_windup[1] = 1; de = -ELEVATOR_MAX-PITCH_SURF_TRIM;}
	else if (de <= -ELEVATOR_MAX-PITCH_SURF_TRIM && e[1] > 0) {anti_windup[1] = 0; de = -ELEVATOR_MAX-PITCH_SURF_TRIM;}  //stop integrating
	else {anti_windup[1] = 1;}

	//eliminate wind-up on altitude integral
	if      (de >= ELEVATOR_MAX-PITCH_SURF_TRIM && e[2] < 0) {anti_windup[2] = 1;}
	else if (de >= ELEVATOR_MAX-PITCH_SURF_TRIM && e[2] > 0) {anti_windup[2] = 0;}  //stop integrating
	else if (de <= -ELEVATOR_MAX-PITCH_SURF_TRIM && e[2] < 0) {anti_windup[2] = 0;}  //stop integrating
	else if (de <= -ELEVATOR_MAX-PITCH_SURF_TRIM && e[2] > 0) {anti_windup[2] = 1;}
	else {anti_windup[2] = 1;}

	return de;  //rad
}

static double speed_control(double speed_ref, double airspeed, double delta_t)
{
	// Speed tracker
	e[3] = speed_ref - airspeed;
	integrator[3] += e[3]*delta_t*anti_windup[3]; // altitude error integral

		// proportional term  + integral term
	dthr = v_gain[0]*e[3] + v_gain[1]*integrator[3];    // Throttle output

	//eliminate wind-up on airspeed integral
	if      (dthr >= THROTTLE_MAX-THROTTLE_TRIM && e[3] < 0) {anti_windup[3] = 1; dthr = THROTTLE_MAX-THROTTLE_TRIM;}
	else if (dthr >= THROTTLE_MAX-THROTTLE_TRIM && e[3] > 0) {anti_windup[3] = 0; dthr = THROTTLE_MAX-THROTTLE_TRIM;}  //stop integrating
	else if (dthr <= THROTTLE_MIN-THROTTLE_TRIM && e[3] < 0) {anti_windup[3] = 0; dthr = THROTTLE_MIN-THROTTLE_TRIM;}  //stop integrating
	else if (dthr <= THROTTLE_MIN-THROTTLE_TRIM && e[3] > 0) {anti_windup[3] = 1; dthr = THROTTLE_MIN-THROTTLE_TRIM;}
	else {anti_windup[3] = 1;}

	return dthr; // non dimensional
}

static double roll_control (double phi_ref, double roll_angle, double rollrate, double delta_t)
{
	// ROLL tracking controller implemented here

	// roll attitude tracker
	e[0] = phi_ref - roll_angle;
	integrator[0] += e[0]*delta_t*anti_windup[0]; //roll error integral (rad)

	//proportional term + integral term              - roll damper term
	da  = roll_gain[0]*e[0] + roll_gain[1]*integrator[0] - roll_gain[2]*rollrate;

	//eliminate windup
	if      (da >= AILERON_MAX-ROLL_SURF_TRIM && e[0] > 0) {anti_windup[0] = 1; da = AILERON_MAX-ROLL_SURF_TRIM;}
	else if (da >= AILERON_MAX-ROLL_SURF_TRIM && e[0] < 0) {anti_windup[0] = 0; da = AILERON_MAX-ROLL_SURF_TRIM;}  //stop integrating
	else if (da <= -AILERON_MAX-ROLL_SURF_TRIM && e[0] > 0) {anti_windup[0] = 0; da = -AILERON_MAX-ROLL_SURF_TRIM;}  //stop integrating
	else if (da <= -AILERON_MAX-ROLL_SURF_TRIM && e[0] < 0) {anti_windup[0] = 1; da = -AILERON_MAX-ROLL_SURF_TRIM;}
	else {anti_windup[0] = 1;}

    return da;
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
	controlData_ptr->h_cmd = 0;	// altitude command
	controlData_ptr->init_alt = 0;

}
