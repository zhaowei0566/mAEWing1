/*! \file ss_control_interface.h
 *	\brief Control law interface header
 *
 *	\details This file declares the standard function prototypes for interfacing with control laws.
 *	\ingroup control_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $XXX $
 */

#ifndef SS_CONTROL_INTERFACE_H_
#define SS_CONTROL_INTERFACE_H_

extern void init_ss01_control(void);
extern void close_ss01_control(void);
extern void reset_ss01_control(void);
extern void get_ss01_control(double* ss_input, double* ss_output);

extern void init_ss02_control(void);
extern void close_ss02_control(void);
extern void reset_ss02_control(void);
extern void get_ss02_control(double* ss_input, double* ss_output);

extern void init_ss03_control(void);
extern void close_ss03_control(void);
extern void reset_ss03_control(void);
extern void get_ss03_control(double* ss_input, double* ss_output);

#endif /* SS_CONTROL_INTERFACE_H_ */
