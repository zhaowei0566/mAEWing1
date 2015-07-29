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
#include "../utils/misc.h"
#include "actuator_interface.h"
#include "../sensors/Rabbit/rabbit_sensor.h"
#include AIRCRAFT_UP1DIR

/// Absolute limits for PWM output.
/// Note that most servos can respond to commands outside this range, but it is unlikely that commands outside this range will be valid.
#define PWMOUT_1MSEC 1000	///< GPT value corresponding to 1 msec PWM
#define PWMOUT_2MSEC 2000	///< GPT value corresponding to 2 msec PWM

/// Arrays of calibration coefficients using macros defined in aircraft/XXX_config.h.
static double dthr_cal[] = PWMOUT_DTHR_CAL;
static double l1_cal[]   = PWMOUT_L1_CAL;
static double l2_cal[]   = PWMOUT_L2_CAL;
static double l3_cal[]   = PWMOUT_L3_CAL;
static double l4_cal[] 	 = PWMOUT_L4_CAL;
static double r1_cal[] 	 = PWMOUT_R1_CAL;
static double r2_cal[] 	 = PWMOUT_R2_CAL;
static double r3_cal[] 	 = PWMOUT_R3_CAL;
static double r4_cal[] 	 = PWMOUT_R4_CAL;

/// Compute order of polynomial calibration (length of array - 1)
static int dthr_ord = sizeof(dthr_cal)/sizeof(*dthr_cal) - 1;
static int l1_ord 	= sizeof(l1_cal)/sizeof(*l1_cal) - 1;
static int l2_ord 	= sizeof(l2_cal)/sizeof(*l2_cal) - 1;
static int l3_ord 	= sizeof(l3_cal)/sizeof(*l3_cal) - 1;
static int l4_ord 	= sizeof(l4_cal)/sizeof(*l4_cal) - 1;
static int r1_ord 	= sizeof(r1_cal)/sizeof(*r1_cal) - 1;
static int r2_ord 	= sizeof(r2_cal)/sizeof(*r2_cal) - 1;
static int r3_ord 	= sizeof(r3_cal)/sizeof(*r3_cal) - 1;
static int r4_ord 	= sizeof(r4_cal)/sizeof(*r4_cal) - 1;

/// commands in counts
uint16_t dthr_cnts = 0;
uint16_t l1_cnts = 0;
uint16_t l2_cnts = 0;
uint16_t l3_cnts = 0;
uint16_t l4_cnts = 0;
uint16_t r1_cnts = 0;
uint16_t r2_cnts = 0;
uint16_t r3_cnts = 0;
uint16_t r4_cnts = 0;

extern void init_actuators(){
}

// Return control outputs based on references and feedback signals.
extern void set_actuators(struct control *controlData_ptr) {
	write_rabbit_sensor(&AMS5812,controlData_ptr);
}
	
void write_rabbit_sensor(cyg_i2c_device* device, struct control *controlData_ptr){	
	uint8_t pack[18];
	int bytesWrite;
	
	// Enforce surface limits and apply calibration
	dthr_cnts 	= polyval(dthr_cal, saturation(controlData_ptr->dthr,THROTTLE_MIN,THROTTLE_MAX),dthr_ord);
	l1_cnts   	= polyval(l1_cal, saturation(controlData_ptr->l1,L1_MIN,L1_MAX),l1_ord);
	l2_cnts   	= polyval(l2_cal, saturation(controlData_ptr->l2,L2_MIN,L2_MAX),l2_ord);
	l3_cnts   	= polyval(l3_cal, saturation(controlData_ptr->l3,L3_MIN,L3_MAX),l3_ord);
	l4_cnts 	= polyval(l4_cal, saturation(controlData_ptr->l4,L4_MIN,L4_MAX),l4_ord);
	r1_cnts 	= polyval(r1_cal, saturation(controlData_ptr->r1,R1_MIN,R1_MAX),r1_ord);
	r2_cnts 	= polyval(r2_cal, saturation(controlData_ptr->r2,R2_MIN,R2_MAX),r2_ord);
	r3_cnts 	= polyval(r3_cal, saturation(controlData_ptr->r3,R3_MIN,R3_MAX),r3_ord);
	r4_cnts 	= polyval(r4_cal, saturation(controlData_ptr->r4,R4_MIN,R4_MAX),r4_ord);	
	
	// pack into buffer
	pack[0]   = (byte) (l1_cnts & 0xff);
	pack[1]   = (byte) ((l1_cnts >> 8) & 0xff);
	pack[2]   = (byte) (l2_cnts & 0xff);
	pack[3]   = (byte) ((l2_cnts >> 8) & 0xff);
	pack[4]   = (byte) (l3_cnts & 0xff);
	pack[5]   = (byte) ((l3_cnts >> 8) & 0xff);
	pack[6]   = (byte) (r1_cnts & 0xff);
	pack[7]   = (byte) ((r1_cnts >> 8) & 0xff);
	pack[8]   = (byte) (l4_cnts & 0xff);
	pack[9]   = (byte) ((l4_cnts >> 8) & 0xff);
	pack[10]  = (byte) (r2_cnts & 0xff);
	pack[11]  = (byte) ((r2_cnts >> 8) & 0xff);
	pack[12]  = (byte) (r3_cnts & 0xff);
	pack[13]  = (byte) ((r3_cnts >> 8) & 0xff);
	pack[14]  = (byte) (r4_cnts & 0xff);
	pack[15]  = (byte) ((r4_cnts >> 8) & 0xff);
	pack[16]  = (byte) (dthr_cnts & 0xff);
	pack[17]  = (byte) ((dthr_cnts >> 8) & 0xff);
	
	// write to Teensy3.1
	bytesWrite = cyg_i2c_tx(device, pack, 18);
}

extern void close_actuators(){
}
