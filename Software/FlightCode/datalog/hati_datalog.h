
#ifndef SKOLL_DATALOG_H_
#define SKOLL_DATALOG_H_
	
// Datalogging setup
#define LOG_ARRAY_SIZE 90000 ///< Number of data points in the logging array. 50 Hz * 60 sec/min * 30 minutes = 90000

#define NUM_DOUBLE_VARS 6	///< Number of variables that will be logged as doubles
#define NUM_FLOAT_VARS 72	///< Number of variables that will be logged as floats
#define NUM_INT_VARS 1		///< Number of variables that will be logged as ints
#define NUM_SHORT_VARS 10	///< Number of variables that will be logged as shorts

//Names of matlab variables MUST match pointers!!

/// char array of variable names for doubles
char* saveAsDoubleNames[NUM_DOUBLE_VARS] = {
		"lat", "lon","alt",
		"navlat", "navlon", "navalt"};

/// double pointer array to variables that will be saved as doubles
double* saveAsDoublePointers[NUM_DOUBLE_VARS] = {
		&gpsData.lat, &gpsData.lon, &gpsData.alt,
		&navData.lat, &navData.lon, &navData.alt};

/// char array of variable names for floats
char* saveAsFloatNames[NUM_FLOAT_VARS] = {
 			"time",
			"p", "q", "r", 
			"ax", "ay", "az", 
			"hx", "hy", "hz",
			"h", "ias", 
			"h_filt","ias_filt",
			"Ps","Pd", 
			"vn", "ve", "vd",
			"navvn", "navve","navvd",
			"phi", "theta", "psi",
			"p_bias", "q_bias", "r_bias",
			"ax_bias", "ay_bias", "az_bias",
			"select_incp","pitch_incp", "mode_incp",
			"roll_incp",
			"accel_lf","accel_lr","accel_cf",
			"accel_cr","accel_rf","accel_rr",
			"l3", "l2", "l1", "r1",
			"l4", "r4", "dthr","r3", "r2",
			"pitch_cmd_pilot","pitch_cmd_excite",
			"roll_cmd_pilot","roll_cmd_excite",
			"phi_cmd","theta_cmd","ias_cmd","h_cmd",
			"init_alt","zdot_cmd",
			"etime_daq", "etime_nav", "etime_guidance",
			"etime_control", "etime_sysid",
			"etime_actuators","etime_datalog","etime_telemetry","etime_mission",
			"gps_speed","gps_course","gps_update"};
								
/// double pointer array to variables that will be saved as floats
double* saveAsFloatPointers[NUM_FLOAT_VARS] = {
			&imuData.time,
			&imuData.p, &imuData.q, &imuData.r,
			&imuData.ax, &imuData.ay, &imuData.az,
			&imuData.hx, &imuData.hy, &imuData.hz,
			&adData.h, &adData.ias, 
			&adData.h_filt, &adData.ias_filt,
			&adData.Ps, &adData.Pd, 
			&gpsData.vn, &gpsData.ve, &gpsData.vd, 
			&navData.vn, &navData.ve, &navData.vd,
			&navData.phi, &navData.the, &navData.psi,
			&navData.gb[0], &navData.gb[1], &navData.gb[2],
			&navData.ab[0], &navData.ab[1], &navData.ab[2],
			&inceptorData.select, &inceptorData.pitch, &inceptorData.mode,
			&inceptorData.roll,
			&accelData.lf, &accelData.lr, &accelData.cf,
			&accelData.cr, &accelData.rf, &accelData.rr,
			&controlData.l3, &controlData.l2, &controlData.l1, &controlData.r1, 
			&controlData.l4, &controlData.r4, &controlData.dthr, &controlData.r3, &controlData.r2,
			&controlData.pitch_cmd_pilot, &controlData.pitch_cmd_excite,
			&controlData.roll_cmd_pilot, &controlData.roll_cmd_excite,
			&controlData.phi_cmd, &controlData.theta_cmd, &controlData.ias_cmd, &controlData.h_cmd,
			&controlData.init_alt, &controlData.zdot_cmd,
			&etime_daq, &etime_nav, &etime_guidance,
			&etime_control, &etime_sysid,
			&etime_actuators, &etime_datalog, &etime_telemetry, &etime_mission,
			&gpsData.speedOverGround,&gpsData.courseOverGround,&gpsData.update};

/// char array of variable names for ints
char* saveAsIntNames[NUM_INT_VARS] = {"imuStatus"};

/// int32_t pointer array to variables that will be saved as ints
int32_t* saveAsIntPointers[NUM_INT_VARS] = {(int32_t *)&imuData.err_type};


/// char array of variable names for shorts
char* saveAsShortNames[NUM_SHORT_VARS] = {"mode", "satVisible", "navValid","cpuLoad","adStatus","run_num","run_excitation",
"claw_mode","claw_select","sysid_select"};

/// uint16_t pointer array to variables that will be saved as shorts
uint16_t* saveAsShortPointers[NUM_SHORT_VARS] = {&missionData.mode, &gpsData.satVisible,
												  &gpsData.navValid,&cpuLoad,&adData.status,&missionData.run_num,&missionData.run_excitation,
												  &missionData.claw_mode,&missionData.claw_select,&missionData.sysid_select};
#endif	

