/*
 * rtc.h
 *
 *  Created on: 25 nov. 2018
 *      Author: Ludo
 */

#ifndef RTC_H
#define RTC_H

#include "neom8n.h"

/*** RTC functions ***/

void RTC_Init(unsigned char rtc_use_lse, unsigned int lsi_freq_hz);
void RTC_ClearAlarmFlags(void);
void RTC_Calibrate(Timestamp* gps_timestamp);
void RTC_GetTimestamp(Timestamp* rtc_timestamp);

#endif /* RTC_H */
