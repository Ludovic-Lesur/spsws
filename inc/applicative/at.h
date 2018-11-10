/*
 * at.h
 *
 *  Created on: 9 nov. 2018
 *      Author: Ludovic
 */

#ifndef AT_H
#define AT_H

#include "neom8n.h"

/*** AT functions ***/

void AT_Init(void);
void AT_PrintGpsTimestamp(GPS_TimestampData* gps_timestamp);
void AT_PrintGpsPosition(GPS_PositionData* gps_position);

#endif /* AT_H */
