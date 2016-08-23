/*! \file telemetry.c
 *	\brief Send telemetry data through serial port
 *
 *	\details
 *	\ingroup telemetry_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: telemetry.c 761 2012-01-19 17:23:49Z murch $
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>
#include <sched.h>
#include <cyg/posix/pthread.h>
#include <cyg/kernel/kapi.h>
#include <cyg/cpuload/cpuload.h>
#include <cyg/io/io.h>
#include <cyg/io/serialio.h>

#include "../globaldefs.h"
#include "../utils/serial_mpc5200.h"
#include "../utils/misc.h"
#include "../extern_vars.h"
#include "telemetry_interface.h"
#include AIRCRAFT_UP1DIR

#define TELE_PACKET_SIZE 31 // 16-bit telemetry packet bytes
#define SEND_PACKET_SIZE (2*TELE_PACKET_SIZE + 5) // 8-bit telemetry packet = [<UUT>, tele_data, <16bit_CKSUM>]
#define STATUS_PACKET_SIZE 103

extern char statusMsg[STATUS_PACKET_SIZE];	

static int port;

void init_telemetry(){
	// Open serial port for send_telemetry. Set in /aircraft/xxx_config.h
	port = open_serial(TELEMETRY_PORT, TELEMETRY_BAUDRATE);	
}

void send_telemetry(struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr, uint16_t cpuLoad)
{
	int bytes=0;
	unsigned short flags=0;
	unsigned long tmp;
	uint16_t tele_data[TELE_PACKET_SIZE], output_CKSUM=0;
	static byte sendpacket[SEND_PACKET_SIZE]={'U','U','T',};

	// Build send_telemetry data packet
	tmp = (unsigned long)( sensorData_ptr->imuData_ptr->time*1e04 );		// time buffer will now overflow after 59.6 hrs (thats what 4 bytes' worth!)
	memcpy(&tele_data[0],&tmp,4);

	//tele_data[2] = (uint16_t)( sensorData_ptr->imuData_ptr->p*R2D / 200.0 * 0x7FFF );	//rate = 200 deg/s max saturation
	tele_data[2] = (uint16_t)( sensorData_ptr->imuData_ptr->q*R2D / 200.0 * 0x7FFF );
	//tele_data[4] = (uint16_t)( sensorData_ptr->imuData_ptr->r*R2D / 200.0 * 0x7FFF );
	
	tele_data[3] = (uint16_t)( sensorData_ptr->imuData_ptr->ax / 100.0 * 0x7FFF );	// IMU accels = 100 m/sec2
	//tele_data[6] = (uint16_t)( sensorData_ptr->imuData_ptr->ay / 100.0 * 0x7FFF );
	tele_data[4] = (uint16_t)( sensorData_ptr->imuData_ptr->az / 100.0 * 0x7FFF );
	
	tele_data[5] = (uint16_t)( (sensorData_ptr->adData_ptr->h) / 1000.0 * 0x7FFF );	// max AGL Alt.(m) = 1000 m
	tele_data[6] = (uint16_t)( (sensorData_ptr->adData_ptr->ias) / 80.0 * 0x7FFF );	// max Indicated Airspeed(IAS) = 80 m/s	

	tele_data[7] = (uint16_t)(navData_ptr->psi*R2D / 180.0 * 0x7FFF );	// Euler angles [psi,theta,phi] (deg)
	tele_data[8]= (uint16_t)(navData_ptr->the*R2D / 90.0 * 0x7FFF );
	tele_data[9]= (uint16_t)(navData_ptr->phi*R2D / 180.0 * 0x7FFF );

	tele_data[10]= (uint16_t)(controlData_ptr->l1*R2D / 45.0 * 0x7FFF );	// control surface commands (normalized 0-1)
	tele_data[11]= (uint16_t)(controlData_ptr->l2*R2D / 45.0 * 0x7FFF );
	tele_data[12]= (uint16_t)(controlData_ptr->l3*R2D / 45.0 * 0x7FFF );
	tele_data[13]= (uint16_t)(controlData_ptr->l4*R2D / 45.0 * 0x7FFF );
	tele_data[14]= (uint16_t)(controlData_ptr->r1*R2D / 45.0 * 0x7FFF );
	tele_data[15]= (uint16_t)(controlData_ptr->r2*R2D / 45.0 * 0x7FFF );
	tele_data[16]= (uint16_t)(controlData_ptr->r3*R2D / 45.0 * 0x7FFF );
	tele_data[17]= (uint16_t)(controlData_ptr->r4*R2D / 45.0 * 0x7FFF );
	tele_data[18]= (uint16_t)(controlData_ptr->dthr * 0x7FFF );
	
	tele_data[19]= (uint16_t)(sensorData_ptr->accelData_ptr->cf / 100.0 * 0x7FFF );	// accelerometers = 8 g = 90 mps2
	tele_data[20]= (uint16_t)(sensorData_ptr->accelData_ptr->cr / 100.0 * 0x7FFF );
	tele_data[21]= (uint16_t)(sensorData_ptr->accelData_ptr->lf / 100.0 * 0x7FFF );
	tele_data[22]= (uint16_t)(sensorData_ptr->accelData_ptr->lr / 100.0 * 0x7FFF );
	tele_data[23]= (uint16_t)(sensorData_ptr->accelData_ptr->rf / 100.0 * 0x7FFF );
	tele_data[24]= (uint16_t)(sensorData_ptr->accelData_ptr->rr / 100.0 * 0x7FFF );
	
	tmp = (unsigned long)(sensorData_ptr->gpsData_ptr->lon *1e07 );
	memcpy(&tele_data[25], &tmp, 4);
	tmp = (unsigned long)(sensorData_ptr->gpsData_ptr->lat *1e07 );
	memcpy(&tele_data[27], &tmp, 4);

	if (missionData_ptr->recording == 1) flags |= 0x01<<0; // Logging
	//if (missionData_ptr->mode == 1) flags |= 0x01<<0;  // Manual mode
	if (missionData_ptr->mode == 2) flags |= 0x01<<1;	// Autopilot mode
	if (missionData_ptr->claw_mode == 1) flags |= 0x01<<2;	// Claw mode = 1
	if (missionData_ptr->claw_mode == 2) flags |= 0x01<<3;	// Claw mode = 2
	if (missionData_ptr->claw_select == 1) flags |= 0x01<<4;	// Claw select = 1
	if (missionData_ptr->claw_select == 2) flags |= 0x01<<5;	// Claw select = 2
	if ( (sensorData_ptr->imuData_ptr->err_type != checksum_err) && (sensorData_ptr->imuData_ptr->err_type != got_invalid) ) flags |= 0x01<<6;
	if (sensorData_ptr->gpsData_ptr->err_type == data_valid || sensorData_ptr->gpsData_ptr->err_type == incompletePacket) flags |= 0x01<<7;
	if (sensorData_ptr->gpsData_ptr->navValid == 0) flags |= 0x01<<8;
	
	tele_data[29] = flags;
	
	tele_data[30] = (uint8_t) cpuLoad << 8 | sensorData_ptr->gpsData_ptr->satVisible;
	
	// Copy tele_data into sendpacket
	memcpy(&sendpacket[3], &tele_data, (2 * TELE_PACKET_SIZE));
	
	// compute 16-bit checksum of output data (excluding the header)
	output_CKSUM = do_chksum(sendpacket, 3, (SEND_PACKET_SIZE - 2));
	
	// Copy Checksum to last 2 bytes of sendpacket
	*(uint16_t *)&sendpacket[(SEND_PACKET_SIZE - 2)] = output_CKSUM;

	// send send_telemetry data packet to serial port
	while(bytes != SEND_PACKET_SIZE) bytes += write(port, &sendpacket[bytes], SEND_PACKET_SIZE-bytes); bytes=0;
	
	// Send status message if present
	if (statusMsg[0] != 0){
		while(bytes != STATUS_PACKET_SIZE) bytes += write(port, &statusMsg[bytes], STATUS_PACKET_SIZE-bytes); bytes=0;
		statusMsg[0] =0;
	}
}
