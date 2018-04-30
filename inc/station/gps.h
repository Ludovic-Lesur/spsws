/*
 * gps.h
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludovic
 */

#ifndef STATION_GPS_H
#define STATION_GPS_H

/*** GPS functions ***/

void GPS_Init(void);
void GPS_FillNmeaRxBuffer(unsigned char new_byte);
unsigned char GPS_Processing(void);

#endif /* STATION_GPS_H */
