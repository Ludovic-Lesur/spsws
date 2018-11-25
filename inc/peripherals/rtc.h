/*
 * rtc.h
 *
 *  Created on: 25 nov. 2018
 *      Author: Ludovic
 */

#ifndef RTC_H
#define RTC_H

#include "neom8n.h"

/*** RTC functions ***/

void RTC_Init(void);
void RTC_Calibrate(GPS_TimestampData* gps_timestamp);

#endif /* RTC_H */
