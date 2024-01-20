/*
 * rtc.h
 *
 *  Created on: 25 nov. 2018
 *      Author: Ludo
 */

#ifndef __RTC_H__
#define __RTC_H__

#include "rcc.h"
#include "types.h"

/*** RTC macros ***/

#define RTC_LOCAL_UTC_OFFSET_WINTER		1
#define RTC_LOCAL_UTC_OFFSET_SUMMER		2
#define RTC_WINTER_TIME_LAST_MONTH		3
#define RTC_WINTER_TIME_FIRST_MONTH		11
#define RTC_NUMBER_OF_HOURS_PER_DAY		24
#define RTC_AFTERNOON_HOUR_THRESHOLD	12

/*** RTC structures ***/

/*!******************************************************************
 * \enum RTC_status_t
 * \brief RTC driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	RTC_SUCCESS = 0,
	RTC_ERROR_NULL_PARAMETER,
	RTC_ERROR_INITIALIZATION_MODE,
	RTC_ERROR_WAKEUP_TIMER_REGISTER_ACCESS,
	// Low level drivers errors.
	RTC_ERROR_BASE_RCC = 0x0100,
	// Last base value.
	RTC_ERROR_BASE_LAST = (RTC_ERROR_BASE_RCC + RCC_ERROR_BASE_LAST)
} RTC_status_t;

/*!******************************************************************
 * \enum RTC_time_t
 * \brief RTC time structure.
 *******************************************************************/
typedef struct {
	// Date.
	uint16_t year;
	uint8_t month;
	uint8_t date;
	// Time.
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
} RTC_time_t;

/*** RTC functions ***/

/*!******************************************************************
 * \fn RTC_status_t RTC_init(void)
 * \brief Init RTC peripheral.
 * \param[in]  	alarm_offset_seconds: Random offset to add to the alarm interrupt.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RTC_status_t RTC_init(uint8_t alarm_offset_seconds);

/*!******************************************************************
 * \fn RTC_status_t RTC_calibrate(RTC_time_t* time)
 * \brief Calibrate RTC calendar.
 * \param[in]  	time: Pointer to the absolute time to set in calendar.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RTC_status_t RTC_calibrate(RTC_time_t* time);

/*!******************************************************************
 * \fn RTC_status_t RTC_get_time(RTC_time_t* time)
 * \brief Get RTC time.
 * \param[in]  	time: Pointer to the current absolute RTC time.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RTC_status_t RTC_get_time(RTC_time_t* time);

/*!******************************************************************
 * \fn volatile uint8_t RTC_get_alarm_a_flag(void)
 * \brief Read RTC alarm A interrupt flag.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		RTC alarm A flag.
 *******************************************************************/
volatile uint8_t RTC_get_alarm_a_flag(void);

/*!******************************************************************
 * \fn void RTC_clear_alarm_a_flag(void)
 * \brief Clear RTC alarm A interrupt flag.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void RTC_clear_alarm_a_flag(void);

/*!******************************************************************
 * \fn volatile uint8_t RTC_get_alarm_b_flag(void)
 * \brief Read RTC alarm B interrupt flag.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		RTC alarm B flag.
 *******************************************************************/
volatile uint8_t RTC_get_alarm_b_flag(void);

/*!******************************************************************
 * \fn void RTC_clear_alarm_b_flag(void)
 * \brief Clear RTC alarm B interrupt flag.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void RTC_clear_alarm_b_flag(void);

/*!******************************************************************
 * \fn uint32_t RTC_get_time_seconds(void)
 * \brief Read MCU operating time in seconds.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Current time in seconds.
 *******************************************************************/
uint32_t RTC_get_time_seconds(void);

/*******************************************************************/
#define RTC_exit_error(error_base) { if (rtc_status != RTC_SUCCESS) { status = (error_base + rtc_status); goto errors; } }

/*******************************************************************/
#define RTC_stack_error(void) { if (rtc_status != RTC_SUCCESS) { ERROR_stack_add(ERROR_BASE_RTC + rtc_status); } }

/*******************************************************************/
#define RTC_stack_exit_error(error_code) { if (rtc_status != RTC_SUCCESS) { ERROR_stack_add(ERROR_BASE_RTC + rtc_status); status = error_code; goto errors; } }

#endif /* __RTC_H__ */
