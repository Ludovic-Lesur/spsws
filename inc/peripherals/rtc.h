/*
 * rtc.h
 *
 *  Created on: 25 nov. 2018
 *      Author: Ludo
 */

#ifndef RTC_H
#define RTC_H

#include "mode.h"
#include "neom8n.h"

/*** RTC functions ***/

void RTC_Reset(void);
void RTC_Init(unsigned char* rtc_use_lse, unsigned int lsi_freq_hz);
volatile unsigned char RTC_GetAlarmAFlag(void);
volatile unsigned char RTC_GetAlarmBFlag(void);
void RTC_ClearAlarmAFlag(void);
void RTC_ClearAlarmBFlag(void);
void RTC_Calibrate(Timestamp* gps_timestamp);
void RTC_GetTimestamp(Timestamp* rtc_timestamp);

#endif /* RTC_H */
