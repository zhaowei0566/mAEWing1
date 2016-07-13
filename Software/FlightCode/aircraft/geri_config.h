
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
#define PWMOUT_DTHR_CAL {520, 1272} ///< linear calibration for throttle
#define PWMOUT_L1_CAL   {-143.626237953301,     -132.783454757918,  -92.0783736935874 , -199.678887862221,  -910.167093563455,   1528.21543448172} ///< cubic calibration for L1
#define PWMOUT_L2_CAL   {-566.407605020075,		1527.14167709886} ///< cubic calibration for L2
#define PWMOUT_L3_CAL   {3.0087206599002,       31.9862866347609,   -579.335957214452,  1524.63162852191} ///< cubic calibration for L3
#define PWMOUT_L4_CAL   {-171.915125136958,     -74.6863892998373,  101.495677716979,   7.42427452657357,   -597.144019497296,   1522.32589726225} ///< cubic calibration for L4
#define PWMOUT_R1_CAL   {-66.953450806647,      191.561896357966,   294.229805819839,   344.887761238374,   932.810034411236,    1526.769801352} ///< cubic calibration for R1
#define PWMOUT_R2_CAL   {29.6004221777595,      78.5612056546868,   610.8154113537,     1526.1000796273} ///< cubic calibration for R2
#define PWMOUT_R3_CAL   {6.97058173567647,      -232.337598812444,  -2.0606830907155,   81.1117529844788,   576.852949132383,    1528.29224510245} ///< cubic calibration for R3
#define PWMOUT_R4_CAL   {149.377096231853,      -86.157508793284,   -10.2517072726338,  99.5351760078058,   590.64980123108,     1527.54971803378} ///< cubic calibration for R4

// Pilot inceptor calibration parameters
#define SELECT_INCP_CAL	{0.000863352048698032, -1.58376998680006} ///<  linear calibration for pilot select inceptor
#define PITCH_INCP_CAL	{0.00130197301142324,	-3.96975198221257}	///<  linear calibration for pilot pitch inceptor
#define MODE_INCP_CAL	{0.000850088662947068, -1.56545206486567} ///<  linear calibration for pilot mode inceptor
#define ROLL_INCP_CAL	{-0.00121277738367002,	3.69435537314005}	///<  linear calibration for pilot roll inceptor

#endif	
