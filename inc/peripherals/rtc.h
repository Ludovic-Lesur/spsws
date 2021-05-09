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
void RTC_Calibrate(Timestamp* gps_timestamp);
void RTC_GetTimestamp(Timestamp* rtc_timestamp);

void RTC_EnableAlarmAInterrupt(void);
void RTC_DisableAlarmAInterrupt(void);
volatile unsigned char RTC_GetAlarmAFlag(void);
void RTC_ClearAlarmAFlag(void);

void RTC_EnableAlarmBInterrupt(void);
void RTC_DisableAlarmBInterrupt(void);
volatile unsigned char RTC_GetAlarmBFlag(void);
void RTC_ClearAlarmBFlag(void);

void RTC_StartWakeUpTimer(unsigned int delay_seconds);
void RTC_StopWakeUpTimer(void);
volatile unsigned char RTC_GetWakeUpTimerFlag(void);
void RTC_ClearWakeUpTimerFlag(void);

#endif /* RTC_H */
