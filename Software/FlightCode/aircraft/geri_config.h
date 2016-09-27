
#ifndef GERI_CONFIG_H_
#define GERI_CONFIG_H_

#define AIRCRAFT_GERI	///< Aircraft name. Used in daq() for sensor processing.

// Downlink telemetry configuration
#define TELEMETRY_PORT 		SERIAL_PORT3	///< Serial port for telemetry()
#define TELEMETRY_BAUDRATE	B115200			///< Baud rate of serial port for telemetry()

// Control Surface Trims
#define THROTTLE_TRIM      	0.72		///< [ND], approximate throttle trim value, 0.72
#define PITCH_SURF_TRIM   	0.0  	///< [rad], approximate elevator trim value, was -0.0872 rad
#define ROLL_SURF_TRIM    	0.0 	///< [rad], approximate aileron trim value

// Altitude Controller Theta Command limits
#define THETA_MAX 	 5.0*D2R // 
#define	THETA_MIN	 -5.0*D2R

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
#define PWMOUT_DTHR_CAL {520, 1272} ///< Calibration for throttle
#define PWMOUT_L1_CAL   { 189.99202340,		-832.95141241,	1524.09636609} ///< Calibration for L1
#define PWMOUT_L2_CAL   { 46.82358528,		-760.93077176,	1526.22224555} ///< Calibration for L2
#define PWMOUT_L3_CAL   { 62.07654383,		-766.78117734,	1525.60480110} ///< Calibration for L3
#define PWMOUT_L4_CAL   { 196.98365492,		-853.28589158,	1520.07877899} ///< Calibration for L4
#define PWMOUT_R1_CAL   { -340.87281001,	974.21294551,	1523.21589323} ///< Calibration for R1
#define PWMOUT_R2_CAL   { -139.44104607,	805.34005395,	1524.56888084} ///< Calibration for R2
#define PWMOUT_R3_CAL   { -135.30664906,	780.16312386,	1505.99839192} ///< Calibration for R3
#define PWMOUT_R4_CAL   { -110.51319380,	770.63215961,	1522.96201081} ///< Calibration for R4

// Pilot inceptor calibration parameters
#define PITCH_INCP_CAL	{0.00130197301142324,	-3.96975198221257}	///<  linear calibration for pilot pitch inceptor
#define ROLL_INCP_CAL	{-0.00121277738367002,	3.69435537314005}	///<  linear calibration for pilot roll inceptor
#define MODE_INCP_CAL	{0.000850088662947068, -1.56545206486567} ///<  linear calibration for pilot mode inceptor
#define SELECT_INCP_CAL	{0.000863352048698032, -1.58376998680006} ///<  linear calibration for pilot select inceptor

#endif	
