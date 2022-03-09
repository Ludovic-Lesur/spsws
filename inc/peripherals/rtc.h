/*
 * rtc.h
 *
 *  Created on: 25 nov. 2018
 *      Author: Ludo
 */

#ifndef RTC_H
#define RTC_H

#include "mode.h"

/*** RTC structures ***/

typedef enum {
	RTC_SUCCESS = 0,
	RTC_ERROR_INITIALIZATION_MODE,
	RTC_ERROR_WAKEUP_TIMER_DELAY,
	RTC_ERROR_WAKEUP_TIMER_RUNNING,
	RTC_ERROR_BASE_LAST = 0x0100
} RTC_status_t;

typedef struct {
	// Date.
	unsigned short year;
	unsigned char month;
	unsigned char date;
	// Time.
	unsigned char hours;
	unsigned char minutes;
	unsigned char seconds;
} RTC_time_t;

/*** RTC functions ***/

void RTC_reset(void);
RTC_status_t RTC_init(unsigned char* rtc_use_lse, unsigned int lsi_freq_hz);
RTC_status_t RTC_calibrate(RTC_time_t* timestamp);
void RTC_get_timestamp(RTC_time_t* timestamp);

void RTC_enable_alarm_a_interrupt(void);
void RTC_disable_alarm_a_interrupt(void);
volatile unsigned char RTC_get_alarm_a_flag(void);
void RTC_clear_alarm_a_flag(void);

void RTC_enable_alarm_b_interrupt(void);
void RTC_disable_alarm_b_interrupt(void);
volatile unsigned char RTC_get_alarm_b_flag(void);
void RTC_clear_alarm_b_flag(void);

RTC_status_t RTC_start_wakeup_timer(unsigned int delay_seconds);
RTC_status_t RTC_stop_wakeup_timer(void);
volatile unsigned char RTC_get_wakeup_timer_flag(void);
void RTC_clear_wakeup_timer_flag(void);

#define RTC_status_check(error_base) { if (rtc_status != RTC_SUCCESS) { status = error_base + rtc_status; goto errors; }}

#endif /* RTC_H */
