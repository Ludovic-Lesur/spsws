/*
 * rtc.c
 *
 *  Created on: 25 nov. 2018
 *      Author: Ludo
 */

#include "rtc.h"

#include "error.h"
#include "exti.h"
#include "nvic.h"
#include "rcc_reg.h"
#include "rtc_reg.h"
#include "types.h"

/*** RTC local macros ***/

#define RTC_INIT_TIMEOUT_COUNT	1000000

/*** RTC local global variables ***/

static volatile uint8_t rtc_alarm_a_flag = 0;
static volatile uint8_t rtc_alarm_b_flag = 0;
static volatile uint32_t rtc_time_seconds = 0;

/*** RTC local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) RTC_IRQHandler(void) {
	// Alarm A interrupt.
	if (((RTC -> ISR) & (0b1 << 8)) != 0) {
		// Set local flag.
		rtc_alarm_a_flag = 1;
		// Clear interrupt flags.
		RTC -> ISR &= ~(0b1 << 8); // ALRAF='0'.
		EXTI_clear_flag(EXTI_LINE_RTC_ALARM);
	}

	// Alarm B interrupt.
	if (((RTC -> ISR) & (0b1 << 9)) != 0) {
		// Set local flag.
		rtc_alarm_b_flag = 1;
		rtc_time_seconds++;
		// Clear interrupt flags.
		RTC -> ISR &= ~(0b1 << 9); // ALRBF='0'.
		EXTI_clear_flag(EXTI_LINE_RTC_ALARM);
	}
}

/*******************************************************************/
static RTC_status_t __attribute__((optimize("-O0"))) _RTC_enter_initialization_mode(void) {
	// Local variables.
	RTC_status_t status = RTC_SUCCESS;
	uint32_t loop_count = 0;
	// Enter key.
	RTC -> WPR = 0xCA;
	RTC -> WPR = 0x53;
	RTC -> ISR |= (0b1 << 7); // INIT='1'.
	// Wait for initialization mode.
	while (((RTC -> ISR) & (0b1 << 6)) == 0) {
		// Wait for INITF='1' or timeout.
		loop_count++;
		if (loop_count > RTC_INIT_TIMEOUT_COUNT) {
			status = RTC_ERROR_INITIALIZATION_MODE;
			break;
		}
	}
	return status;
}

/*******************************************************************/
static void __attribute__((optimize("-O0"))) _RTC_exit_initialization_mode(void) {
	RTC -> ISR &= ~(0b1 << 7); // INIT='0'.
}

/*** RTC functions ***/

/*******************************************************************/
RTC_status_t RTC_init(uint8_t alarm_offset_seconds) {
	// Local variables.
	RTC_status_t status = RTC_SUCCESS;
	RCC_status_t rcc_status = RCC_SUCCESS;
	RCC_clock_t rtc_clock = RCC_CLOCK_LSE;
	uint32_t rtc_clock_hz = 0;
	uint8_t lse_status = 0;
	uint8_t tens = 0;
	uint8_t units = 0;
	// Select peripheral clock.
	RCC -> CSR &= ~(0b11 << 16); // Reset bits 16-17.
	// Get LSE status.
	rcc_status = RCC_get_status(RCC_CLOCK_LSE, &lse_status);
	RCC_exit_error(LPTIM_ERROR_BASE_RCC);
	// Check LSE status.
	if (lse_status != 0) {
		// Use LSE.
		RCC -> CSR |= (0b01 << 16); // RTCSEL='01'.
		rtc_clock = RCC_CLOCK_LSE;
	}
	else {
		// Use LSI.
		RCC -> CSR |= (0b10 << 16); // RTCSEL='10'.
		rtc_clock = RCC_CLOCK_LSI;
	}
	// Get clock source frequency.
	rcc_status = RCC_get_frequency_hz(rtc_clock, &rtc_clock_hz);
	RCC_exit_error(LPTIM_ERROR_BASE_RCC);
	// Enable RTC and register access.
	RCC -> CSR |= (0b1 << 18); // RTCEN='1'.
	// Enter initialization mode.
	status = _RTC_enter_initialization_mode();
	if (status != RTC_SUCCESS) goto errors;
	// Compute prescaler.
	RTC -> PRER = (127 << 16) | (((rtc_clock_hz >> 7) - 1) << 0);
	// Configure alarm A to wake-up MCU every hour.
	tens = ((alarm_offset_seconds % 60) / 10) & 0x07;
	units = ((alarm_offset_seconds % 60) - (tens * 10)) % 0x0F;
	RTC -> ALRMAR |= (0b1 << 31) | (0b1 << 23) | (tens << 4) | (units << 0); // Mask day and hour (only check minutes and seconds).
	RTC -> CR |= (0b1 << 8); // Enable Alarm A.
	// Configure alarm B to wake-up MCU every seconds (watchdog reload and wind measurements for CM).
	RTC -> ALRMBR |= (0b1 << 31) | (0b1 << 23) | (0b1 << 15) | (0b1 << 7); // Mask all fields.
	RTC -> CR |= (0b1 << 9); // Enable Alarm B.
	// Clear flags.
	RTC -> ISR &= 0xFFFF005F;
	// Configure interrupt.
	EXTI_configure_line(EXTI_LINE_RTC_ALARM, EXTI_TRIGGER_RISING_EDGE);
	NVIC_enable_interrupt(NVIC_INTERRUPT_RTC, NVIC_PRIORITY_RTC);
	// Enable alarms.
	RTC -> CR = 0x00003324;
errors:
	_RTC_exit_initialization_mode();
	return status;
}

/*******************************************************************/
RTC_status_t __attribute__((optimize("-O0"))) RTC_calibrate(RTC_time_t* time) {
	// Local variables.
	RTC_status_t status = RTC_SUCCESS;
	uint32_t tr_value = 0;
	uint32_t dr_value = 0;
	uint8_t tens = 0;
	uint8_t units = 0;
	// Check parameters.
	if (time == NULL) {
		status = RTC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Year.
	tens = ((time -> year) - 2000) / 10;
	units = ((time -> year) - 2000) - (tens * 10);
	dr_value |= (tens << 20) | (units << 16);
	// Month.
	tens = (time -> month) / 10;
	units = (time -> month) - (tens * 10);
	dr_value |= (tens << 12) | (units << 8);
	// Date.
	tens = (time -> date) / 10;
	units = (time -> date) - (tens * 10);
	dr_value |= (tens << 4) | (units << 0);
	// Hour.
	tens = (time -> hours) / 10;
	units = (time -> hours) - (tens * 10);
	tr_value |= (tens << 20) | (units << 16);
	// Minutes.
	tens = (time -> minutes) / 10;
	units = (time -> minutes) - (tens * 10);
	tr_value |= (tens << 12) | (units << 8);
	// Seconds.
	tens = (time -> seconds) / 10;
	units = (time -> seconds) - (tens * 10);
	tr_value |= (tens << 4) | (units << 0);
	// Enter initialization mode.
	status = _RTC_enter_initialization_mode();
	if (status != RTC_SUCCESS) goto errors;
	// Perform update.
	RTC -> TR = tr_value;
	RTC -> DR = dr_value;
	// Exit initialization mode and restart RTC.
	_RTC_exit_initialization_mode();
errors:
	return status;
}

/*******************************************************************/
RTC_status_t __attribute__((optimize("-O0"))) RTC_get_time(RTC_time_t* time) {
	// Local variables.
	RTC_status_t status = RTC_SUCCESS;
	uint32_t dr_value = 0;
	uint32_t tr_value = 0;
	// Check parameters.
	if (time == NULL) {
		status = RTC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Read registers.
	dr_value = (RTC -> DR) & 0x00FFFF3F; // Mask reserved bits.
	tr_value = (RTC -> TR) & 0x007F7F7F; // Mask reserved bits.
	// Parse registers into time structure.
	time -> year = 2000 + ((dr_value & (0b1111 << 20)) >> 20) * 10 + ((dr_value & (0b1111 << 16)) >> 16);
	time -> month = ((dr_value & (0b1 << 12)) >> 12) * 10 + ((dr_value & (0b1111 << 8)) >> 8);
	time -> date = ((dr_value & (0b11 << 4)) >> 4) * 10 + (dr_value & 0b1111);
	time -> hours = ((tr_value & (0b11 << 20)) >> 20) * 10 + ((tr_value & (0b1111 << 16)) >> 16);
	time -> minutes = ((tr_value & (0b111 << 12)) >> 12) * 10 + ((tr_value & (0b1111 << 8)) >> 8);
	time -> seconds = ((tr_value & (0b111 << 4)) >> 4) * 10 + (tr_value & 0b1111);
errors:
	return status;
}

/*******************************************************************/
uint8_t RTC_get_alarm_a_flag(void) {
	return rtc_alarm_a_flag;
}

/*******************************************************************/
void RTC_clear_alarm_a_flag(void) {
	rtc_alarm_a_flag = 0;
}

/*******************************************************************/
uint8_t RTC_get_alarm_b_flag(void) {
	return rtc_alarm_b_flag;
}

/*******************************************************************/
void RTC_clear_alarm_b_flag(void) {
	rtc_alarm_b_flag = 0;
}

/*******************************************************************/
uint32_t RTC_get_time_seconds(void) {
	return rtc_time_seconds;
}
