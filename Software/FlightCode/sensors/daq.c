/*! \file daq.c
 *	\brief Data acquisition source code
 *
 *	\details This file implements the init_daq() and get_daq() functions for the UAV.
 *	\ingroup daq_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: daq.c 1014 2014-01-15 18:54:42Z brtaylor $
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <cyg/posix/pthread.h>
#include <cyg/kernel/kapi.h>
#include <cyg/cpuload/cpuload.h>
#include <cyg/gpt/mpc5xxx_gpt.h>
#include <cyg/io/mpc5xxx_gpio.h>
#include <cyg/io/i2c_mpc5xxx.h>
#include <cyg/io/io.h>
#include <cyg/io/serialio.h>

#include "../globaldefs.h"
#include "../utils/serial_mpc5200.h"
#include "../utils/misc.h"
#include "../utils/scheduling.h"
#include "../extern_vars.h"
#include "AirData/airdata_constants.h"
#include "Rabbit/rabbit_interface.h"
#include "IMU/imu_interface.h"
#include "GPIO/gpio_interface.h"
#include "../navigation/nav_interface.h"
#include "daq_interface.h"
#include AIRCRAFT_UP1DIR

/// Arrays of calibration coefficients using macros defined in aircraft/XXX_config.h.
static double incp_select_cal[] = SELECT_INCP_CAL;
static double incp_pitch_cal[] 	= PITCH_INCP_CAL;
static double incp_mode_cal[] 	= MODE_INCP_CAL;
static double incp_roll_cal[] 	= ROLL_INCP_CAL;

/// Compute order of polynomial calibration (length of array - 1)
static int incp_select_ord 	= sizeof(incp_select_cal)/sizeof(*incp_select_cal) - 1;
static int incp_pitch_ord   = sizeof(incp_pitch_cal)/sizeof(*incp_pitch_cal) - 1;
static int incp_mode_ord   	= sizeof(incp_mode_cal)/sizeof(*incp_mode_cal) - 1;
static int incp_roll_ord 	= sizeof(incp_roll_cal)/sizeof(*incp_roll_cal) - 1;

/// Low Pass Filter for speed and altitude signals initialization
static double u_alt[2] = {0,0}; //input of altitude low pass filter { u(k), u(k-1) }
static double y_alt[2] = {0,0}; //output of altitude low pass filter { y(k), y(k-1) }

static double u_speed[2] = {0,0}; //input of altitude low pass filter { u(k), u(k-1) }
static double y_speed[2] = {0,0}; //output of altitude low pass filter { y(k), y(k-1) }

double lp_filter(double signal, double *u, double *y)
{
	const int m=1;  //m = order of denominator of low pass filter

	u[m] = signal;

	y[m] = 0.9802*y[m-1] + 0.0198*u[m-1];	// these coefficients come from a discretized low pass filter with a pole at 2 rad/sec

	u[m-1] = u[m];		// initialize past values for next frame
	y[m-1] = y[m];

	return y[m];
}


void init_daq(struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr)
{
	/* Initialize sensors */
	init_imu();		/* IMU */
}

void get_daq(struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr){		

	// latching for GPS soft start
	static int t0_latched = FALSE;
	static double t0 = 0;

	// local pointers to keep things tidy
	struct imu *imuData_ptr = sensorData_ptr->imuData_ptr;
	struct gps *gpsData_ptr = sensorData_ptr->gpsData_ptr;
	struct airdata *adData_ptr = sensorData_ptr->adData_ptr;
	struct inceptor *inceptorData_ptr = sensorData_ptr->inceptorData_ptr;
	struct rabbit *rabbitData_ptr = sensorData_ptr->rabbitData_ptr;
	struct accel *accelData_ptr = sensorData_ptr->accelData_ptr;

	// IMU Sensor
	read_imu(imuData_ptr);

	// read the rabbit
	read_rabbit(rabbitData_ptr);

	/********** Air Data **********/
	// get the static and dynamic pressure
	adData_ptr->Ps = rabbitData_ptr->Ps;
	adData_ptr->Pd = rabbitData_ptr->Pd;
	
	// Compute pressure altitude; bias removal results in AGL altitude
	adData_ptr->h = AIR_DATA_K1*(1-pow((adData_ptr->Ps)/AIR_DATA_P0,AIR_DATA_K2)) - adData_ptr->bias[0]; //[ m ]
	
	// Compute Indicated Airspeed (IAS). This equation accounts for compressibility effects. Sensor bias is removed prior to calculation
	adData_ptr->ias = copysign(AIR_DATA_K3*sqrt(fabs(pow(fabs((adData_ptr->Pd-adData_ptr->bias[1])/AIR_DATA_P0 +1),AIR_DATA_K4)-1)),adData_ptr->Pd); 

    // Filter altitude and airspeed signals
	adData_ptr->h_filt = lp_filter(adData_ptr->h, u_alt, y_alt);  	    // filtered ALTITUDE
	adData_ptr->ias_filt   = lp_filter(adData_ptr->ias, u_speed, y_speed);	// filtered AIRSPEED
	/********** End Air Data **********/
	
	/********** Accel Data **********/
	// wing accels
	accelData_ptr->rf = -0.002686896961612*rabbitData_ptr->rf + 76.3400996473874;
	accelData_ptr->rr = -0.002693532609047*rabbitData_ptr->rr + 76.0961355496262;
	accelData_ptr->cf = -0.002675203261056*rabbitData_ptr->cf + 75.9817798096145;
	accelData_ptr->lf = -0.002664620875213*rabbitData_ptr->lf + 76.5457536998365;
	accelData_ptr->lr = -0.002657911529701*rabbitData_ptr->lr + 78.1873966679877;
	accelData_ptr->cr = -0.002693529953637*rabbitData_ptr->cr + 77.7648470255344;
	/********** End Accel Data **********/
	
	/********** PWM Data **********/
	// Apply calibration equations
	inceptorData_ptr->select = 		polyval(incp_select_cal, (double)rabbitData_ptr->pwm_ch_three,incp_select_ord);
	inceptorData_ptr->pitch = 		polyval(incp_pitch_cal, (double)rabbitData_ptr->pwm_ch_four,incp_pitch_ord);
	inceptorData_ptr->mode = 		polyval(incp_mode_cal, (double)rabbitData_ptr->pwm_ch_two,incp_mode_ord);
	inceptorData_ptr->roll = 		polyval(incp_roll_cal, (double)rabbitData_ptr->pwm_ch_one,incp_roll_ord);
	/********** End PWM Data **********/
	
	/********** GPIO **********/
	if(rabbitData_ptr->mode == 1){
		if(missionData_ptr->mode==1){
			missionData_ptr->run_num++;
		}
		missionData_ptr->mode = 2; // autopilot
	}
	else{
		missionData_ptr->mode = 1; // manual
	}
	
	// Read GPIOs data dump
	read_gpio(missionData_ptr);
	/********** End GPIO **********/
	
	/********** GPS Data **********/
	gpsData_ptr->newData = rabbitData_ptr->isUpdated;
	gpsData_ptr->satVisible = rabbitData_ptr->satVisible;
	
	if(gpsData_ptr->satVisible>4){
		if(t0_latched == FALSE){
			t0 = imuData_ptr->time;
			t0_latched = TRUE;
		}
		if((imuData_ptr->time-t0)>20){
			gpsData_ptr->navValid = 0;
		}
	}
	else{
		gpsData_ptr->navValid = 1;
	}
	gpsData_ptr->update = 0;
	if(gpsData_ptr->newData == 1){
		gpsData_ptr->update = 1;
		gpsData_ptr->cur_time = imuData_ptr->time;
		gpsData_ptr->lat = rabbitData_ptr->lat;
		gpsData_ptr->lon = rabbitData_ptr->lon;
		gpsData_ptr->alt = rabbitData_ptr->alt;
		gpsData_ptr ->courseOverGround = rabbitData_ptr->courseOverGround;
		gpsData_ptr ->speedOverGround = rabbitData_ptr->speedOverGround;
		gpsData_ptr->vn = cos(gpsData_ptr ->courseOverGround)*gpsData_ptr ->speedOverGround;
		gpsData_ptr->ve = sin(gpsData_ptr ->courseOverGround)*gpsData_ptr ->speedOverGround;
		gpsData_ptr->vd = -1*(gpsData_ptr->alt - gpsData_ptr->alt_prev)/(gpsData_ptr->cur_time - gpsData_ptr->prev_time);
		gpsData_ptr->alt_prev = gpsData_ptr->alt;
		gpsData_ptr->prev_time = gpsData_ptr->cur_time;
	}
	/********** End GPS Data **********/
}
