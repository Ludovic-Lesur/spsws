/*
 * neom8n.h
 *
 *  Created on: 11 aug 2018
 *      Author: Ludo
 */

#ifndef NEOM8N_H
#define NEOM8N_H

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
} NEOM8N_time_t;

typedef struct {
	// Latitude.
	unsigned char lat_degrees;
	unsigned char lat_minutes;
	unsigned int lat_seconds; // = (fractionnal part of minutes * 100000).
	unsigned char lat_north_flag; // 0='S', 1='N'.
	// Longitude.
	unsigned char long_degrees;
	unsigned char long_minutes;
	unsigned int long_seconds; // = (fractionnal part of minutes * 100000).
	unsigned char long_east_flag; // 0='O', 1='E'.
	// Altitude.
	unsigned int altitude;
} NEOM8N_position_t;

typedef enum {
	NEOM8N_SUCCESS,
	NEOM8N_ERROR_TIMEOUT,
	NEOM8N_ERROR_LAST
} NEOM8N_status_t;

/*** NEOM8N user functions ***/

void NEOM8N_init(void);
void NEOM8N_switch_dma_buffer(unsigned char lf_flag);
NEOM8N_status_t NEOM8N_get_time(NEOM8N_time_t* gps_timestamp, unsigned int timeout_seconds, unsigned int supercap_voltage_min_mv);
NEOM8N_status_t NEOM8N_get_position(NEOM8N_position_t* gps_position, unsigned int timeout_seconds, unsigned int supercap_voltage_min_mv, unsigned int* fix_duration_seconds);

#endif /* NEOM8N_H */
