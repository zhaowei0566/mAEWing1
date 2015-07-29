
#ifndef AIRDATA_INTERFACE_H_
#define AIRDATA_INTERFACE_H_


/// Auxiliary function to estimate biases on air data sensors. Implemented in airdata_bias.c
void airdata_bias_estimate(struct airdata *adData_ptr	///< pointer to adData structure
		);

#endif

