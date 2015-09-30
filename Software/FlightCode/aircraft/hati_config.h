
#ifndef HATI_CONFIG_H_
#define HATI_CONFIG_H_

#define AIRCRAFT_HATI	///< Aircraft name. Used in daq() for sensor processing.

// Downlink telemetry configuration
#define TELEMETRY_PORT 		SERIAL_PORT3	///< Serial port for telemetry()
#define TELEMETRY_BAUDRATE	B115200			///< Baud rate of serial port for telemetry()

// Control Surface Trims
#define THROTTLE_TRIM      	0.72		///< [ND], approximate throttle trim value
#define PITCH_SURF_TRIM   	-0.0872	///< [rad], approximate elevator trim value
#define ROLL_SURF_TRIM    	0.0 	///< [rad], approximate aileron trim value


// Control Surface limits, max and min
#define THROTTLE_MAX 	 1.0
#define	THROTTLE_MIN	 0.0
#define	L1_MAX		 0.6109	///< [rad], 35deg -- L1 Flutter Suppression Surface
#define	L1_MIN	    -0.6109 ///< [rad],-35deg -- L1 Flutter Suppression Surface
#define	L2_MAX		 0.6109	///< [rad], 35deg -- L2 Flutter Suppression Surface
#define	L2_MIN	    -0.6109 ///< [rad],-35deg -- L2 Flutter Suppression Surface
#define	L3_MAX		 0.6109	///< [rad], 35deg -- L3 Flutter Suppression Surface
#define	L3_MIN	    -0.6109 ///< [rad],-35deg -- L3 Flutter Suppression Surface
#define L4_MAX	 	 0.6109	///< [rad], 35deg -- L4 Flutter Suppression Surface
#define L4_MIN		-0.6109 ///< [rad],-35deg -- L4 Flutter Suppression Surface
#define R1_MAX	 	 0.6109	///< [rad], 35deg -- R1 Flutter Suppression Surface
#define R1_MIN		-0.6109 ///< [rad],-35deg -- R1 Flutter Suppression Surface
#define R2_MAX	 	 0.6109	///< [rad], 35deg -- R2 Flutter Suppression Surface
#define R2_MIN		-0.6109 ///< [rad],-35deg -- R2 Flutter Suppression Surface
#define R3_MAX	 	 0.6109	///< [rad], 35deg -- R3 Flutter Suppression Surface
#define R3_MIN		-0.6109 ///< [rad],-35deg -- R3 Flutter Suppression Surface
#define R4_MAX	 	 0.6109	///< [rad], 35deg -- R4 Flutter Suppression Surface
#define R4_MIN		-0.6109	///< [rad],-35deg -- R4 Flutter Suppression Surface

// MPC5200 PWM output command calibration parameters
#define PWMOUT_DTHR_CAL {1000, 1000} ///< linear calibration for throttle
#define PWMOUT_L1_CAL   {-238.92381412565,	-916.614725709856,	1527.08196276848} ///< cubic calibration for L1
#define PWMOUT_L2_CAL   {39.9649209160205,	-598.214251832176,	1520.83292060929} ///< cubic calibration for L2
#define PWMOUT_L3_CAL   {-2.1225918226342,	-579.300940846605,	1524.26942535826} ///< cubic calibration for L3
#define PWMOUT_L4_CAL   {25.7770363400901,	-597.440318056541,	1522.07058671221} ///< cubic calibration for L4
#define PWMOUT_R1_CAL   {326.732206887716,	944.528590817906,	1523.53221072802} ///< cubic calibration for R1
#define PWMOUT_R2_CAL   {123.862765296962,	616.841901119626,	1519.04379527984} ///< cubic calibration for R2
#define PWMOUT_R3_CAL   {84.9321713451011,	625.903161119561,	1489.05005722093} ///< cubic calibration for R3
#define PWMOUT_R4_CAL   {122.582161052627,	596.231384481085,	1520.4578871808} ///< cubic calibration for R4

// Pilot inceptor calibration parameters
#define SELECT_INCP_CAL	{0.000863352048698032,	-1.58376998680006}	///<  linear calibration for pilot select inceptor
#define PITCH_INCP_CAL	{0.00168804085122093,	-5.12677951000987}	///<  linear calibration for pilot pitch inceptor
#define MODE_INCP_CAL	{0.000850088662947068,	-1.56545206486567}	///<  linear calibration for pilot mode inceptor
#define ROLL_INCP_CAL	{-0.00168875943697853,	 5.12507563684683}	///<  linear calibration for pilot roll inceptor

#endif	
