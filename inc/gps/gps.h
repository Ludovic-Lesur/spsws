/*
 * gps.h
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludovic
 */

#ifndef GPS_GPS_H
#define GPS_GPS_H

/*** GPS macros ***/

#define GPS_PERIOD_DAYS_MIN_VALUE		1 // GPS minimum position fix period in days.
#define GPS_PERIOD_DAYS_DEFAULT_VALUE	1 // Default GPS position fix period (in case if NVM read failure).
#define GPS_PERIOD_DAYS_MAX_VALUE		7 // GPS maximum position fix period in days.

/*** GPS functions ***/

void GPS_Processing(void);
#ifdef HARDWARE_TIMER
void GPS_GetStatusByte(unsigned char* gps_status_byte);
#endif

#endif /* GPS_GPS_H */
