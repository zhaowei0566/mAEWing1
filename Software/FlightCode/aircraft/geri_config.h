
#ifndef GERI_CONFIG_H_
#define GERI_CONFIG_H_

#define AIRCRAFT_GERI	///< Aircraft name. Used in daq() for sensor processing.

// Downlink telemetry configuration
#define TELEMETRY_PORT 		SERIAL_PORT3	///< Serial port for telemetry()
#define TELEMETRY_BAUDRATE	B115200			///< Baud rate of serial port for telemetry()

// Control Surface Trims
#define THROTTLE_TRIM      	0.72		///< [ND], approximate throttle trim value
#define PITCH_SURF_TRIM   	-0.0872	///< [rad], approximate elevator trim value
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
#define PWMOUT_L1_CAL   { 23.65731,	-78.34749,	770.0316,	1548.055} ///< Calibration for L1
#define PWMOUT_L2_CAL   { -23.64406,	0.7737799,	723.8292,	1553.741} ///< Calibration for L2
#define PWMOUT_L3_CAL   { 310.5530,	-148.8107,	719.6825,	1553.219} ///< Calibration for L3
#define PWMOUT_L4_CAL   { 153.0319,	-178.0020,	812.2671,	1544.434} ///< Calibration for L4
#define PWMOUT_R1_CAL   { 47.91341,	265.7748,	-892.1562,	1537.289} ///< Calibration for R1
#define PWMOUT_R2_CAL   { -61.02339,	149.4639,	-782.3937,	1541.148} ///< Calibration for R2
#define PWMOUT_R3_CAL   { -20.95530,	154.2702,	-767.2924,	1538.620} ///< Calibration for R3
#define PWMOUT_R4_CAL   { 155.3547,	-732.6522,	1536.338} ///< Calibration for R4

// Pilot inceptor calibration parameters
#define SELECT_INCP_CAL	{0.000863352048698032, -1.58376998680006} ///<  linear calibration for pilot select inceptor
#define PITCH_INCP_CAL	{0.00130197301142324,	-3.96975198221257}	///<  linear calibration for pilot pitch inceptor
#define MODE_INCP_CAL	{0.000850088662947068, -1.56545206486567} ///<  linear calibration for pilot mode inceptor
#define ROLL_INCP_CAL	{-0.00121277738367002,	3.69435537314005}	///<  linear calibration for pilot roll inceptor

#endif	
