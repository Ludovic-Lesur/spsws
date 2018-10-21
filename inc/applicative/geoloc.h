/*
 * geoloc.h
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludovic
 */

#ifndef APPLICATIVE_GEOLOC_H
#define APPLICATIVE_GEOLOC_H

/*** GEOLOC macros ***/

#define GEOLOC_PERIOD_DAYS_MIN_VALUE		1 // GEOLOC minimum position fix period in days.
#define GEOLOC_PERIOD_DAYS_DEFAULT_VALUE	1 // Default GEOLOC position fix period (in case if NVM read failure).
#define GEOLOC_PERIOD_DAYS_MAX_VALUE		7 // GEOLOC maximum position fix period in days.

/*** GEOLOC functions ***/

void GEOLOC_Processing(void);
void GEOLOC_GetStatusByte(unsigned char* gps_status_byte);

#endif /* APPLICATIVE_GEOLOC_H */
