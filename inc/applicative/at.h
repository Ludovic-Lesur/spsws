/*
 * at.h
 *
 *  Created on: 9 nov. 2018
 *      Author: Ludo
 */

#ifndef AT_H
#define AT_H

#include "mode.h"

#ifdef ATM

/*** AT functions ***/

void AT_Init(void);
void AT_Task(void);
void AT_FillRxBuffer(unsigned char rx_byte);
void AT_PrintTestResult(unsigned char status, int rssi);
void AT_PrintRain(unsigned char rain_edge_count);
void AT_PrintWindSpeed(unsigned int wind_speed_mh);
void AT_PrintWindDirection(unsigned int wind_direction_degrees, int wind_direction_x, int wind_direction_y);

#endif

#endif /* AT_H */
