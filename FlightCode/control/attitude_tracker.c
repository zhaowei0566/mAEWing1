/*! \file baseline_control.c
 *	\brief baseline controller source code
 *
 *	\details The baseline controller is a classical PI design that tracks pitch angle (theta) and roll angle (phi).
 *	A yaw damper and roll damper is used for the lateral axis, and a pitch damper is used for the longitudinal axis.
 *
 *	\ingroup control_fcns
 *
 *	\author University of Minnesota
 *	\author Aerospace Engineering and Mechanics
 *	\copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: baseline_control.c 929 2012-10-29 16:50:59Z joh07594 $
 */

#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "control_interface.h"



// ***********************************************************************************

//#include "../aircraft/thor_config.h"  // for SIL_Sim
#include AIRCRAFT_UP1DIR            // for Flight Code

// ***********************************************************************************

/// Definition of local functions: ****************************************************
static double roll_control (double phi_ref, double roll_angle, double rollrate, double delta_t);
static double pitch_control(double the_ref, double pitch, double pitchrate, double delta_t);
static double speed_control(double speed_ref, double airspeed, double delta_t);

// initialize pitch and roll angle tracking errors, integrators, and anti wind-up operators
// 0 values correspond to the roll tracker, 1 values correspond to the theta tracker
static double e[3] = {0,0,0};
static double integrator[3] = {0,0,0};
static short anti_windup[3]={1,1,1};   // integrates when anti_windup is 1

/// ****************************************************************************************
/// Roll and pitch angle digital controller - parameters and variables
#ifdef AIRCRAFT_FENRIR
	static double roll_gain[3]  = {-0.64,-0.20,-0.07};  // PI gains for roll tracker and roll damper
	static double pitch_gain[3] = {-0.90,-0.30,-0.08};  // PI gains for theta tracker and pitch damper
	static double v_gain[2] 	= {0.091, 0.020};		// PI gains for speed tracker
#endif
static double da; // Delta aileron
static double de; // Delta elevator
static double dthr;
/// *****************************************************************************************

extern void get_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr) {
/// Return control outputs based on references and feedback signals.

#ifdef AIRCRAFT_FENRIR
	double base_pitch_cmd= 0.1222;  	// (Trim value) 7 deg
#endif

	double phi   = navData_ptr->phi;
	double theta = navData_ptr->the - base_pitch_cmd; //subtract theta trim value to convert to delta coordinates
	double p     = sensorData_ptr->imuData_ptr->p; // Roll rate
	double q     = sensorData_ptr->imuData_ptr->q; // Pitch rate
	double ias   = sensorData_ptr->adData_ptr->ias_filt;

    double phi_cmd = controlData_ptr->phi_cmd;
    double theta_cmd = controlData_ptr->theta_cmd;
	double ias_cmd = controlData_ptr->ias_cmd;
	
	controlData_ptr->dthr = speed_control(ias_cmd, ias, TIMESTEP);
    controlData_ptr->de   = pitch_control(theta_cmd, theta, q, TIMESTEP);
    controlData_ptr->da   = roll_control(phi_cmd, phi, p, TIMESTEP);
	controlData_ptr->l1   = 0;		// L1 [rad]
    controlData_ptr->r1   = 0; 		// R1 [rad]
	controlData_ptr->l4   = 0; 		// L4 [rad]
	controlData_ptr->r4   = 0; 		// R4 [rad]
}

// Roll get_control law: angles in radians. Rates in rad/s. Time in seconds
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
	else if (da <= AILERON_MIN-ROLL_SURF_TRIM && e[0] > 0) {anti_windup[0] = 0; da = AILERON_MIN-ROLL_SURF_TRIM;}  //stop integrating
	else if (da <= AILERON_MIN-ROLL_SURF_TRIM && e[0] < 0) {anti_windup[0] = 1; da = AILERON_MIN-ROLL_SURF_TRIM;}
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

// Reset parameters to initial values
extern void reset_control(struct control *controlData_ptr){

	integrator[0] = integrator[1] = 0;
	anti_windup[0] = anti_windup[1] = 1;
	e[0] = e[1] = 0;

	controlData_ptr->dthr = 0; 	// throttle
	controlData_ptr->de   = 0; 	// elevator
	controlData_ptr->da   = 0; 	// rudder
	controlData_ptr->l1   = 0;	// L1 [rad]
    controlData_ptr->r1   = 0; 	// R1 [rad]
	controlData_ptr->l4   = 0; 	// L4 [rad]
	controlData_ptr->r4   = 0; 	// R4 [rad]
}
