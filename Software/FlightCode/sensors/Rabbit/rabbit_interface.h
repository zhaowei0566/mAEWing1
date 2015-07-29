
#ifndef RABBIT_INTERFACE_H_
#define RABBIT_INTERFACE_H_

void init_rabbit(struct rabbit *rabbitData_ptr	///< pointer to gpsData structure
		);

void read_rabbit(struct rabbit *rabbitData_ptr	///< pointer to rabbitData structure
		);
		
/// Local generic function for reading AMS pressure sensors. Returns 1 if successful, 0 otherwise.
void	read_rabbit_sensor(cyg_i2c_device* device,	///< Pointer to I2C device structure
struct rabbit *rabbitData_ptr
);

#endif /* RABBIT_INTERFACE_H_ */
