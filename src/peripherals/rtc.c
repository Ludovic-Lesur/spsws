/*
 * rtc.c
 *
 *  Created on: 25 nov. 2018
 *      Author: Ludo
 */

#include "rtc.h"

#include "exti.h"
#include "exti_reg.h"
#include "nvic.h"
#include "rcc_reg.h"
#include "rtc_reg.h"
#include "types.h"

/*** RTC local macros ***/

#define RTC_INIT_TIMEOUT_COUNT		1000
#define RTC_WAKEUP_TIMER_DELAY_MAX	65536

/*** RTC local global variables ***/

static volatile uint8_t rtc_alarm_b_flag = 0;
static volatile uint8_t rtc_wakeup_timer_flag = 0;

/*** RTC local functions ***/

/* RTC INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) RTC_IRQHandler(void) {
	// Alarm B interrupt.
	if (((RTC -> ISR) & (0b1 << 9)) != 0) {
		// Set local flag.
		if (((RTC -> CR) & (0b1 << 13)) != 0) {
			rtc_alarm_b_flag = 1;
		}
		// Clear flags.
		RTC -> ISR &= ~(0b1 << 9); // ALRBF='0'.
		EXTI -> PR |= (0b1 << EXTI_LINE_RTC_ALARM);
	}
	// Wake-up timer interrupt.
	if (((RTC -> ISR) & (0b1 << 10)) != 0) {
		// Set local flag.
		if (((RTC -> CR) & (0b1 << 14)) != 0) {
			rtc_wakeup_timer_flag = 1;
		}
		// Clear flags.
		RTC -> ISR &= ~(0b1 << 10); // WUTF='0'.
		EXTI -> PR |= (0b1 << EXTI_LINE_RTC_WAKEUP_TIMER);
	}
}

/* ENTER INITIALIZATION MODE TO ENABLE RTC REGISTERS UPDATE.
 * @param:			None.
 * @return status:	Function execution status.
 */
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

/* EXIT INITIALIZATION MODE TO PROTECT RTC REGISTERS.
 * @param:	None.
 * @return:	None.
 */
static void __attribute__((optimize("-O0"))) _RTC_exit_initialization_mode(void) {
	RTC -> ISR &= ~(0b1 << 7); // INIT='0'.
}

/*** RTC functions ***/

/* RESET RTC PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) RTC_reset(void) {
	// Local variables.
	uint8_t j = 0;
	// Reset RTC peripheral.
	RCC -> CSR |= (0b1 << 19); // RTCRST='1'.
	for (j=0 ; j<100 ; j++);
	RCC -> CSR &= ~(0b1 << 19); // RTCRST='0'.
}

/* INIT HARDWARE RTC PERIPHERAL.
 * @param rtc_use_lse:			RTC will be clocked on LSI if 0, on LSE otherwise.
 * @param lsi_freq_hz:			Effective LSI oscillator frequency used to compute the accurate prescaler value (only if LSI is used as source).
 * @param alarm_offset_seconds:	Offset added to the alarm A in seconds.
 * @return status:				Function execution status.
 */
RTC_status_t __attribute__((optimize("-O0"))) RTC_init(uint8_t* rtc_use_lse, uint32_t lsi_freq_hz, uint8_t alarm_offset_seconds) {
	// Local variables.
	RTC_status_t status = RTC_SUCCESS;
	uint8_t tens = 0;
	uint8_t units = 0;
	// Check parameters.
	if (rtc_use_lse == NULL) {
		status = RTC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Manage RTC clock source.
	if ((*rtc_use_lse) != 0) {
		RCC -> CSR |= (0b01 << 16); // RTCSEL='01' (LSE).
	}
	else {
		RCC -> CSR |= (0b10 << 16); // RTCSEL='10' (LSI).
	}
	// Enable RTC and register access.
	RCC -> CSR |= (0b1 << 18); // RTCEN='1'.
	// Switch to LSI if RTC failed to enter initialization mode.
	status = _RTC_enter_initialization_mode();
	if (status != RTC_SUCCESS) {
		// Try using LSI.
		RTC_reset();
		RCC -> CSR |= (0b10 << 16); // RTCSEL='10'.
		RCC -> CSR |= (0b1 << 18); // RTCEN='1'.
		status = _RTC_enter_initialization_mode();
		if (status != RTC_SUCCESS) {
			// Update flag.
			(*rtc_use_lse) = 0;
			goto errors;
		}
	}
	// Configure prescaler.
	if ((*rtc_use_lse) != 0) {
		// LSE frequency is 32.768kHz typical.
		RTC -> PRER = (127 << 16) | (255 << 0); // PREDIV_A=127 and PREDIV_S=255 (128*256 = 32768).
	}
	else {
		// Compute prescaler according to measured LSI frequency.
		RTC -> PRER = (127 << 16) | (((lsi_freq_hz / 128) - 1) << 0); // PREDIV_A=127 and PREDIV_S=((lsi_freq_hz/128)-1).
	}
	// Force registers reset.
	RTC -> CR = 0;
	RTC -> ALRMAR = 0;
	RTC -> ALRMBR = 0;
	// Bypass shadow registers.
	RTC -> CR |= (0b1 << 5); // BYPSHAD='1'.
	// Configure alarm A to wake-up MCU every hour.
	tens = ((alarm_offset_seconds % 60) / 10) & 0x07;
	units = ((alarm_offset_seconds % 60) - (tens * 10)) % 0x0F;
	RTC -> ALRMAR |= (0b1 << 31) | (0b1 << 23) | (tens << 4) | (units << 0); // Mask day and hour (only check minutes and seconds).
	RTC -> CR |= (0b1 << 8); // Enable Alarm A.
	// Configure alarm B to wake-up every seconds (watchdog reload and wind measurements for CM).
	RTC -> ALRMBR |= (0b1 << 31) | (0b1 << 23) | (0b1 << 15) | (0b1 << 7); // Mask all fields.
	RTC -> CR |= (0b1 << 9); // Enable Alarm B.
	// Configure wake-up timer.
	RTC -> CR |= (0b100 << 0); // Wake-up timer clocked by RTC clock (1Hz).
	_RTC_exit_initialization_mode();
	// Configure EXTI lines.
	EXTI_configure_line(EXTI_LINE_RTC_ALARM, EXTI_TRIGGER_RISING_EDGE);
	EXTI_configure_line(EXTI_LINE_RTC_WAKEUP_TIMER, EXTI_TRIGGER_RISING_EDGE);
	// Disable interrupts and clear all flags.
	RTC -> ISR &= 0xFFFE0000;
	EXTI -> PR |= (0b1 << EXTI_LINE_RTC_ALARM);
	EXTI -> PR |= (0b1 << EXTI_LINE_RTC_WAKEUP_TIMER);
	// Set interrupt priority.
	NVIC_set_priority(NVIC_INTERRUPT_RTC, 1);
	NVIC_enable_interrupt(NVIC_INTERRUPT_RTC);
errors:
	return status;
}

/* UPDATE RTC CALENDAR WITH A GPS TIMESTAMP.
 * @param time:	Pointer to time from GPS.
 * @return status:		Function execution status.
 */
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

/* GET CURRENT RTC TIME.
 * @param rtc_time:	Pointer to time that will contain current RTC time.
 * @return status:	Function execution status.
 */
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

/* RETURN THE CURRENT ALARM INTERRUPT STATUS.
 * @param:	None.
 * @return:	1 if the RTC interrupt occured, 0 otherwise.
 */
volatile uint8_t RTC_get_alarm_a_flag(void) {
	// Local variables.
	volatile uint8_t rtc_alarm_a_flag = 0;
	// Read register.
	rtc_alarm_a_flag = (((RTC -> ISR) & (0b1 << 8)) != 0) ? 1 : 0;
	return rtc_alarm_a_flag;
}

/* CLEAR ALARM A INTERRUPT FLAG.
 * @param:	None.
 * @return:	None.
 */
void RTC_clear_alarm_a_flag(void) {
	// Clear all flags.
	RTC -> ISR &= ~(0b1 << 8); // ALRAF='0'.
	EXTI -> PR |= (0b1 << EXTI_LINE_RTC_ALARM);
}

/* ENABLE RTC ALARM B INTERRUPT.
 * @param:	None.
 * @return:	None.
 */
void RTC_enable_alarm_b_interrupt(void) {
	// Enable interrupt.
	RTC -> CR |= (0b1 << 13); // ALRBIE='1'.
}

/* DISABLE RTC ALARM A INTERRUPT.
 * @param:	None.
 * @return:	None.
 */
void RTC_disable_alarm_b_interrupt(void) {
	// Disable interrupt.
	RTC -> CR &= ~(0b1 << 13); // ALRBIE='0'.
}

/* RETURN THE CURRENT ALARM INTERRUPT STATUS.
 * @param:	None.
 * @return:	1 if the RTC interrupt occured, 0 otherwise.
 */
volatile uint8_t RTC_get_alarm_b_flag(void) {
	return rtc_alarm_b_flag;
}

/* CLEAR ALARM A INTERRUPT FLAG.
 * @param:	None.
 * @return:	None.
 */
void RTC_clear_alarm_b_flag(void) {
	// Clear all flags.
	RTC -> ISR &= ~(0b1 << 9); // ALRBF='0'.
	EXTI -> PR |= (0b1 << EXTI_LINE_RTC_ALARM);
	rtc_alarm_b_flag = 0;
}

/* START RTC WAKE-UP TIMER.
 * @param delay_seconds:	Delay in seconds.
 * @return status:			Function execution status.
 */
RTC_status_t RTC_start_wakeup_timer(uint32_t delay_seconds) {
	// Local variables.
	RTC_status_t status = RTC_SUCCESS;
	// Check parameter.
	if (delay_seconds > RTC_WAKEUP_TIMER_DELAY_MAX) {
		status = RTC_ERROR_WAKEUP_TIMER_DELAY;
		goto errors;
	}
	// Check if timer si not allready running.
	if (((RTC -> CR) & (0b1 << 10)) != 0) {
		status = RTC_ERROR_WAKEUP_TIMER_RUNNING;
		goto errors;
	}
	// Enable RTC and register access.
	status = _RTC_enter_initialization_mode();
	if (status != RTC_SUCCESS) goto errors;
	// Configure wake-up timer.
	RTC -> WUTR = (delay_seconds - 1);
	// Clear flags.
	RTC_clear_wakeup_timer_flag();
	// Enable interrupt.
	RTC -> CR |= (0b1 << 14); // WUTE='1'.
	// Start timer.
	RTC -> CR |= (0b1 << 10); // Enable wake-up timer.
	_RTC_exit_initialization_mode();
errors:
	return status;
}

/* STOP RTC WAKE-UP TIMER.
 * @param:			None.
 * @return status:	Function execution status.
 */
RTC_status_t RTC_stop_wakeup_timer(void) {
	// Local variables.
	RTC_status_t status = RTC_SUCCESS;
	// Enable RTC and register access.
	status = _RTC_enter_initialization_mode();
	if (status != RTC_SUCCESS) goto errors;
	// Disable wake-up timer.
	RTC -> CR &= ~(0b1 << 10);
	_RTC_exit_initialization_mode();
	// Disable interrupt.
	RTC -> CR &= ~(0b1 << 14); // WUTE='0'.
errors:
	return status;
}

/* RETURN THE CURRENT ALARM INTERRUPT STATUS.
 * @param:	None.
 * @return:	1 if the RTC interrupt occured, 0 otherwise.
 */
volatile uint8_t RTC_get_wakeup_timer_flag(void) {
	return rtc_wakeup_timer_flag;
}

/* CLEAR ALARM A INTERRUPT FLAG.
 * @param:	None.
 * @return:	None.
 */
void RTC_clear_wakeup_timer_flag(void) {
	// Clear flag.
	RTC -> ISR &= ~(0b1 << 10); // WUTF='0'.
	EXTI -> PR |= (0b1 << EXTI_LINE_RTC_WAKEUP_TIMER);
	rtc_wakeup_timer_flag = 0;
}
