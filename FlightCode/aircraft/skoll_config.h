
#ifndef SKOLL_CONFIG_H_
#define SKOLL_CONFIG_H_

#define AIRCRAFT_SKOLL	///< Aircraft name. Used in daq() for sensor processing.

// GPS Sensor Configuration
#define GPS_PORT 		SERIAL_PORT2	///< Serial port for GPS receiver, used in init_daq()
#define GPS_BAUDRATE	B115200			///< Baud rate of serial port for GPS receiver (crescent), used in init_daq()

// Downlink telemetry configuration
#define TELEMETRY_PORT 		SERIAL_PORT3	///< Serial port for telemetry()
#define TELEMETRY_BAUDRATE	B115200			///< Baud rate of serial port for telemetry()

// Control Surface Trims -- PLACEHOLDERS
// TO DO -- DEFINE TRIMS FOR ALL FLUTTER SUPRESSION SURFACES
#define THROTTLE_TRIM      0.72		///< [ND], approximate throttle trim value
#define PITCH_SURF_TRIM   -0.0872	///< [rad], approximate elevator trim value
#define ROLL_SURF_TRIM    0.0 	///< [rad], approximate aileron trim value


// Control Surface limits, max and min
#define THROTTLE_MAX 	 1.0
#define	THROTTLE_MIN	 0.0
#define	ELEVATOR_MAX	 0.6109	///< [rad], 35deg
#define	ELEVATOR_MIN	-0.6109 ///< [rad],-35deg
#define AILERON_MAX	 0.6109	///< [rad], 35deg
#define AILERON_MIN	-0.6109 ///< [rad],-35deg
#define	L1_MAX		 0.6109	///< [rad], 35deg -- L1 Flutter Suppression Surface
#define	L1_MIN	        -0.6109 ///< [rad],-35deg -- L1 Flutter Suppression Surface
#define R1_MAX	 	 0.6109	///< [rad], 35deg -- R1 Flutter Suppression Surface
#define R1_MIN		-0.6109 ///< [rad],-35deg -- R1 Flutter Suppression Surface
#define L4_MAX	 	 0.6109	///< [rad], 35deg -- L4 Flutter Suppression Surface
#define L4_MIN		-0.6109 ///< [rad],-35deg -- L4 Flutter Suppression Surface
#define R4_MAX	 	 0.6109	///< [rad], 35deg -- R4 Flutter Suppression Surface
#define R4_MIN		-0.6109	///< [rad],-35deg -- R4 Flutter Suppression Surface

// MPC5200 PWM output channel assignments
#define PWMOUT_DTHR_CH  0 ///<  PWM output channel for throttle
#define PWMOUT_DE_CH  	1 ///<  PWM output channel for elevator
#define PWMOUT_DA_CH 	3 ///<  PWM output channel for left aileron
#define PWMOUT_L1_CH 	2 ///<  PWM output channel for L1 Flutter Suppression Surface
#define PWMOUT_R1_CH 	7 ///<  PWM output channel for R1 Flutter Suppression Surface
#define PWMOUT_L4_CH 	5 ///<  PWM output channel for L4 Flutter Suppression Surface
#define PWMOUT_R4_CH	4 ///<  PWM output channel for R4 Flutter Suppression Surface

// MPC5200 PWM output command calibration parameters
#define PWMOUT_DTHR_CAL {2720, 2720} ///< linear calibration for throttle, pwm 1.292ms (motor off) to 1.800ms (100% pwr), from ESC data. Note motor on value is 1.307ms Register values from MPC Servo Calibration.xls
#define PWMOUT_DE_CAL  	{-6.85159967976556,	1536.43791765489,	4136.32259505696} ///< calibration for elevator
#define PWMOUT_DA_CAL   {40.3938273657162,	-1530.84847587925,	4128.05315614618} ///< calibration for aileron
#define PWMOUT_L1_CAL   {-265.573489186189,	-2190.68158467737,	4097.98813549916} ///< cubic calibration for L1
#define PWMOUT_R1_CAL   {301.742881096192,	2329.6476985925,	4094.62569585923} ///< cubic calibration for right R1
#define PWMOUT_L4_CAL   {25.3145068984323,	-1649.2165450339,	4134.66225412391} ///< calibration for left L4
#define PWMOUT_R4_CAL   {90.0240481839605,	1591.32761218294,	4129.90418118467} ///< calibration for right R4

// Pilot inceptor channels
#define THR_INCP_CH		1 ///<  input channel for pilot throttle inceptor
#define PITCH_INCP_CH	2 ///<  input channel for pilot pitch inceptor
#define YAW_INCP_CH		0 ///<  input channel for pilot yaw inceptor
#define ROLL_INCP_CH	3 ///<  input channel for pilot roll inceptor

// Pilot inceptor calibration parameters
#define PWMIN_SCALING 10000 	///< scaling parameter to apply to PWM readings prior to applying calibration
#define THR_INCP_CAL	{6.27781267245976, -1.41309798036129}	///<  linear calibration for pilot throttle inceptor
#define PITCH_INCP_CAL	{-111.529886096001,	96.8294668116000,	-21.5238268564487,	0.763949002413993}	///<  linear calibration for pilot pitch inceptor
#define YAW_INCP_CAL	{84.7756193670826,	-76.8786260771467,	17.1754736690082,	-0.535104948804940}	///<  linear calibration for pilot yaw inceptor
#define ROLL_INCP_CAL	{-42.3502385103704,	40.4850850262976,	-6.72738032331522,	-0.496271142735041}	///<  linear calibration for pilot roll inceptor

#endif	
