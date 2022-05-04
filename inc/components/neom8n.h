/*
 * neom8n.h
 *
 *  Created on: 11 aug 2018
 *      Author: Ludo
 */

#ifndef NEOM8N_H
#define NEOM8N_H

#include "lptim.h"
#include "lpuart.h"
#include "math.h"
#include "rtc.h"
#include "string.h"

/*** NEOM8N structures ***/

typedef enum {
	NEOM8N_SUCCESS = 0,
	NEOM8N_ERROR_TIMEOUT,
	NEOM8N_ERROR_CHECKSUM_INDEX,
	NEOM8N_ERROR_CHECKSUM,
	NEOM8N_ERROR_NMEA_FIELD_LENGTH,
	NEOM8N_ERROR_NMEA_MESSAGE,
	NEOM8N_ERROR_NMEA_NORTH_FLAG,
	NEOM8N_ERROR_NMEA_EAST_FLAG,
	NEOM8N_ERROR_NMEA_UNIT,
	NEOM8N_ERROR_TIME_INVALID,
	NEOM8N_ERROR_TIME_TIMEOUT,
	NEOM8N_ERROR_POSITION_INVALID,
	NEOM8N_ERROR_POSITION_TIMEOUT,
	NEOM8N_ERROR_BASE_LPUART = 0x0100,
	NEOM8N_ERROR_BASE_LPTIM = (NEOM8N_ERROR_BASE_LPUART + LPUART_ERROR_BASE_LAST),
	NEOM8N_ERROR_BASE_RTC = (NEOM8N_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
	NEOM8N_ERROR_BASE_STRING = (NEOM8N_ERROR_BASE_RTC + RTC_ERROR_BASE_LAST),
	NEOM8N_ERROR_BASE_LAST = (NEOM8N_ERROR_BASE_STRING + STRING_ERROR_BASE_LAST)
} NEOM8N_status_t;

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

/*** NEOM8N user functions ***/

void NEOM8N_init(void);
void NEOM8N_switch_dma_buffer(unsigned char lf_flag);
NEOM8N_status_t NEOM8N_get_time(RTC_time_t* gps_time, unsigned int timeout_seconds);
NEOM8N_status_t NEOM8N_get_position(NEOM8N_position_t* gps_position, unsigned int timeout_seconds, unsigned int* fix_duration_seconds);

#define NEOM8N_status_check(error_base) { if (neom8n_status != NEOM8N_SUCCESS) { status = error_base + neom8n_status; goto errors; }}
#define NEOM8N_error_check() { ERROR_status_check(neom8n_status, NEOM8N_SUCCESS, ERROR_BASE_NEOM8N); }
#define NEOM8N_error_check_print() { ERROR_status_check_print(neom8n_status, NEOM8N_SUCCESS, ERROR_BASE_NEOM8N); }

#endif /* NEOM8N_H */
