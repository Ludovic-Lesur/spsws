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

void RTC_reset(void);
void RTC_init(unsigned char* rtc_use_lse, unsigned int lsi_freq_hz);
void RTC_calibrate(NEOM8N_time_t* gps_timestamp);
void RTC_get_timestamp(NEOM8N_time_t* rtc_timestamp);

void RTC_enable_alarm_a_interrupt(void);
void RTC_disable_alarm_a_interrupt(void);
volatile unsigned char RTC_get_alarm_a_flag(void);
void RTC_clear_alarm_a_flag(void);

void RTC_enable_alarm_b_interrupt(void);
void RTC_disable_alarm_b_interrupt(void);
volatile unsigned char RTC_get_alarm_b_flag(void);
void RTC_clear_alarm_b_flag(void);

void RTC_start_wakeup_timer(unsigned int delay_seconds);
void RTC_stop_wakeup_timer(void);
volatile unsigned char RTC_get_wakeup_timer_flag(void);
void RTC_clear_wakeup_timer_flag(void);

#endif /* RTC_H */
