/*
 * neom8n.h
 *
 *  Created on: 11 aug 2018
 *      Author: Ludo
 */

#ifndef NEOM8N_H
#define NEOM8N_H

/*** NEOM8N macros ***/

#define NMEA_CR		'\r'
#define NMEA_LF		'\n'

/*** NEOM8N structures ***/

typedef struct {
	// Date.
	unsigned short year;
	unsigned char month;
	unsigned char date;
	// Time.
	unsigned char hours;
	unsigned char minutes;
	unsigned char seconds;
#ifdef IM_HWT
	// Absolute time (since MCU start-up) in seconds when GPS timestamp is retrieved (used to know MCU start-up timestamp).
	unsigned char mcu_time_seconds;
#endif
} Timestamp;

typedef struct {
	// Latitude.
	unsigned char lat_degrees;
	unsigned char lat_minutes;
	unsigned int lat_seconds; // = (fractionnal part of minutes * 100000).
	unsigned char lat_north; // 0='S', 1='N'.
	// Longitude.
	unsigned char long_degrees;
	unsigned char long_minutes;
	unsigned int long_seconds; // = (fractionnal part of minutes * 100000).
	unsigned char long_east; // 0='O', 1='E'.
	// Altitude.
	unsigned int altitude;
} Position;

typedef enum {
	NEOM8N_SUCCESS,			// Parsing successful and data valid.
	NEOM8N_INVALID_DATA,	// Parsing successful but data invalid.
	NEOM8N_TIMEOUT			// Parsing failure (= timeout).
} NEOM8N_ReturnCode;

/*** NEOM8N user functions ***/

void NEOM8N_Init(void);
NEOM8N_ReturnCode NEOM8N_GetTimestamp(Timestamp* gps_timestamp, unsigned char timeout_seconds);
unsigned char NEOM8N_TimestampIsValid(Timestamp* local_gps_timestamp);
NEOM8N_ReturnCode NEOM8N_GetPosition(Position* gps_position, unsigned char timeout_seconds);

/*** NEOM8N utility functions ***/

void NEOM8N_SwitchDmaBuffer(unsigned char lf_flag);

#endif /* NEOM8N_H */
