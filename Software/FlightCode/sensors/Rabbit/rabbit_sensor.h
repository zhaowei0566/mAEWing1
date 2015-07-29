
#ifndef RABBIT_SENSOR_H_
#define RABBIT_SENSOR_H_

#define P_MIN_COUNTS  3277 	 ///< 10% of full scale digital range
#define P_MAX_COUNTS  29491  ///< 90% of full scale digital range
	
/// Teensy
static	cyg_i2c_device AMS5812 = {
		.i2c_bus		  = &mpc5200_i2c_bus0,
		.i2c_address	  = 0x07,
		.i2c_flags      = 0x00,
		.i2c_delay      = CYG_I2C_DEFAULT_DELAY
};

#endif /* RABBIT_SENSOR_H_ */
