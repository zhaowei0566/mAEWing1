
#ifndef SKOLL_CONFIG_H_
#define SKOLL_CONFIG_H_

#define AIRCRAFT_SKOLL	///< Aircraft name. Used in daq() for sensor processing.

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
#define PWMOUT_L1_CAL   {-99.802083579952,	-786.92868369903,	1493.14035210883} ///< cubic calibration for L1
#define PWMOUT_L2_CAL   {7.62121145643319,	-559.488672646819,	1517.56933320333} ///< cubic calibration for L2
#define PWMOUT_L3_CAL   {28.5188844973309,	-573.868390336405,	1519.99703637294} ///< cubic calibration for L3
#define PWMOUT_L4_CAL   {-14.2465106358729,	-603.544546567549,	1458.98403324584} ///< cubic calibration for L4
#define PWMOUT_R1_CAL   {131.520087353901,	829.949968718669,	1481.53897522002} ///< cubic calibration for R1
#define PWMOUT_R2_CAL   {109.808187889774,	582.618397498397,	1518.5969945643} ///< cubic calibration for R2
#define PWMOUT_R3_CAL   {70.6150703129649,	577.841439589334,	1525.5774784222} ///< cubic calibration for R3
#define PWMOUT_R4_CAL   {50.0946571141479,	606.03678699808,	1516.88467486882} ///< cubic calibration for R4

// Pilot inceptor calibration parameters
#define SELECT_INCP_CAL	{0.000863352048698032,	-1.58376998680006}	///<  linear calibration for pilot select inceptor
#define PITCH_INCP_CAL	{0.00168804085122093,	-5.12677951000987}	///<  linear calibration for pilot pitch inceptor
#define MODE_INCP_CAL	{0.000850088662947068,	-1.56545206486567}	///<  linear calibration for pilot mode inceptor
#define ROLL_INCP_CAL	{-0.00168875943697853,	 5.12507563684683}	///<  linear calibration for pilot roll inceptor

#endif	
