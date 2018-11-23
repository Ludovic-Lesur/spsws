/*
 * neom8n.h
 *
 *  Created on: 11 aug 2018
 *      Author: Ludovic
 */

#ifndef COMPONENTS_NEOM8N_H_
#define COMPONENTS_NEOM8N_H_

/*** NEOM8N macros ***/

#define NMEA_CR		'\r'
#define NMEA_LF		'\n'

/*** NEOM8N structures ***/

typedef struct {
	// Date.
	unsigned char date_day;
	unsigned char date_month;
	unsigned short date_year;
	// Time.
	unsigned char time_hours;
	unsigned char time_minutes;
	unsigned char time_seconds;
	// Absolute time (since MCU start-up) in seconds when GPS timestamp is retrieved (used to know MCU start-up timestamp).
	unsigned char mcu_time_seconds;
} GPS_TimestampData;

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
} GPS_PositionData;

typedef enum {
	NEOM8N_SUCCESS,			// Parsing successful and data valid.
	NEOM8N_INVALID_DATA,	// Parsing successfuul but data invalid.
	NEOM8N_TIMEOUT			// Parsing failure (= timeout).
} NEOM8N_ReturnCode;

/*** NEOM8N functions ***/

void NEOM8N_Init(void);
void NEOM8N_StopRx(void);
NEOM8N_ReturnCode NEOM8N_GetTimestamp(GPS_TimestampData* gps_timestamp, unsigned char timeout_seconds);
unsigned char NEOM8N_TimestampIsValid(GPS_TimestampData local_gps_timestamp);
NEOM8N_ReturnCode NEOM8N_GetPosition(GPS_PositionData* gps_position, unsigned char timeout_seconds);
void NEOM8N_SwitchDmaBuffer(void);

#endif /* COMPONENTS_NEOM8N_H_ */
