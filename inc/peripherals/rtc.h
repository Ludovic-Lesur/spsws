/*
 * rtc.h
 *
 *  Created on: 25 nov. 2018
 *      Author: Ludo
 */

#ifndef RTC_H
#define RTC_H

#include "mode.h"

/*** RTC macros ***/

#define RTC_LOCAL_UTC_OFFSET_WINTER		1
#define RTC_LOCAL_UTC_OFFSET_SUMMER		2
#define RTC_WINTER_TIME_LAST_MONTH		3
#define RTC_WINTER_TIME_FIRST_MONTH		11
#define RTC_NUMBER_OF_HOURS_PER_DAY		24
#define RTC_AFTERNOON_HOUR_THRESHOLD	12

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
RTC_status_t __attribute__((optimize("-O0"))) RTC_init(unsigned char* rtc_use_lse, unsigned int lsi_freq_hz, unsigned char alarm_offset_seconds);
RTC_status_t __attribute__((optimize("-O0"))) RTC_calibrate(RTC_time_t* timestamp);
void __attribute__((optimize("-O0"))) RTC_get_timestamp(RTC_time_t* timestamp);

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
#define RTC_error_check() { ERROR_status_check(rtc_status, RTC_SUCCESS, ERROR_BASE_RTC); }
#define RTC_error_check_print() { ERROR_status_check(rtc_status, RTC_SUCCESS, ERROR_BASE_RTC); }

#endif /* RTC_H */
