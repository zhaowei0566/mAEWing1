
#ifndef ACTUATOR_INTERFACE_H_
#define ACTUATOR_INTERFACE_H_

extern void init_actuators();

extern void set_actuators(struct control * controlData_ptr ///< Pointer to control data structure
		);
		
void	write_rabbit_sensor(cyg_i2c_device* device,	///< Pointer to I2C device structure
struct control * controlData_ptr 
);

extern void close_actuators();

#endif /* ACTUATOR_INTERFACE_H_ */
