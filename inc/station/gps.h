/*
 * gps.h
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludovic
 */

#ifndef STATION_GPS_H
#define STATION_GPS_H

#include "lpuart.h" // For USE_DMA flag.

/*** GPS macros ***/

#define NMEA_CR		'\r'
#define NMEA_LF		'\n'

/*** GPS functions ***/

#ifdef USE_DMA
void GPS_SwitchDmaBuffer(void);
#else
void GPS_FillNmeaRxBuffer(unsigned char new_byte);
#endif
unsigned char GPS_Processing(unsigned char* sigfox_data, unsigned char sigfox_data_length);

#endif /* STATION_GPS_H */
