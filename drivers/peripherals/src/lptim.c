/*
 * lptim.c
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludo
 */

#include "lptim.h"

#include "error.h"
#include "error_base.h"
#include "exti.h"
#include "iwdg.h"
#include "lptim_reg.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "types.h"

/*** LPTIM local macros ***/

#define LPTIM_TIMEOUT_COUNT		1000000

#define LPTIM_ARR_MAX_VALUE		0xFFFF

#define LPTIM_DELAY_MS_MIN		2
#define LPTIM_DELAY_MS_MAX		((LPTIM_ARR_MAX_VALUE * 1000) / (lptim_ctx.clock_frequency_hz))

/*** LPTIM local structures ***/

/*******************************************************************/
typedef struct {
	RCC_clock_t clock_source;
	uint32_t clock_frequency_hz;
	volatile uint8_t wake_up;
} LPTIM_context_t;

/*** LPTIM local global variables ***/

static LPTIM_context_t lptim_ctx;

/*** LPTIM local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) LPTIM1_IRQHandler(void) {
	// Check flag.
	if (((LPTIM1 -> ISR) & (0b1 << 1)) != 0) {
		// Set local flag.
		if (((LPTIM1 -> IER) & (0b1 << 1)) != 0) {
			lptim_ctx.wake_up = 1;
		}
		// Clear flag.
		LPTIM1 -> ICR |= (0b1 << 1);
	}
	EXTI_clear_flag(EXTI_LINE_LPTIM1);
}

/*** LPTIM functions ***/

/*******************************************************************/
LPTIM_status_t __attribute__((optimize("-O0"))) LPTIM1_init(void) {
	// Local variables.
	LPTIM_status_t status = LPTIM_SUCCESS;
	RCC_status_t rcc_status = RCC_SUCCESS;
	uint8_t lse_status = 0;
	uint32_t lptim_clock_hz = 0;
	// Get LSE status.
	rcc_status = RCC_get_status(RCC_CLOCK_LSE, &lse_status);
	RCC_exit_error(LPTIM_ERROR_BASE_RCC);
	// Update clock source.
	lptim_ctx.clock_source = (lse_status != 0) ? RCC_CLOCK_LSE : RCC_CLOCK_LSI;
	// Get clock source frequency.
	rcc_status = RCC_get_frequency_hz(lptim_ctx.clock_source, &lptim_clock_hz);
	RCC_exit_error(LPTIM_ERROR_BASE_RCC);
	lptim_ctx.clock_frequency_hz = (lptim_clock_hz >> 3);
	// Enable LPTIM EXTI line.
	EXTI_configure_line(EXTI_LINE_LPTIM1, EXTI_TRIGGER_RISING_EDGE);
errors:
	return status;
}

/*******************************************************************/
LPTIM_status_t __attribute__((optimize("-O0"))) LPTIM1_delay_milliseconds(uint32_t delay_ms, LPTIM_delay_mode_t delay_mode) {
	// Local variables.
	LPTIM_status_t status = LPTIM_SUCCESS;
	uint32_t arr = 0;
	uint32_t loop_count = 0;
	// Check delay.
	if ((delay_ms > LPTIM_DELAY_MS_MAX) || (delay_ms > (IWDG_FREE_DELAY_SECONDS_MAX * 1000))) {
		status = LPTIM_ERROR_DELAY_OVERFLOW;
		goto errors;
	}
	if (delay_ms < LPTIM_DELAY_MS_MIN) {
		status = LPTIM_ERROR_DELAY_UNDERFLOW;
		goto errors;
	}
	// Force APB clock to access registers.
	RCC -> CCIPR &= ~(0b11 << 18); // LPTIM1SEL='00'.
	// Enable peripheral clock.
	RCC -> APB1ENR |= (0b1 << 31); // LPTIM1EN='1'.
	// Configure peripheral.
	LPTIM1 -> CFGR |= (0b011 << 9); // Prescaler = 8.
	LPTIM1 -> IER |= (0b1 << 1); // ARRMIE='1'.
	// Reset flags.
	LPTIM1 -> ICR |= (0b1 << 4) | (0b1 << 1);
	// Enable peripheral.
	LPTIM1 -> CR |= (0b1 << 0); // ENABLE='1'.
	// Compute ARR value.
	arr = (LPTIM1 -> ARR);
	arr &= 0xFFFF0000;
	arr |= ((((delay_ms - 1) * lptim_ctx.clock_frequency_hz) / (1000)) & 0x0000FFFF);
	// Write register.
	LPTIM1 -> ARR = arr;
	// Wait for ARR write operation to complete.
	while (((LPTIM1 -> ISR) & (0b1 << 4)) == 0) {
		loop_count++;
		if (loop_count > LPTIM_TIMEOUT_COUNT) {
			status = LPTIM_ERROR_ARR_TIMEOUT;
			goto errors;
		}
	}
	// Select clock source.
	switch (lptim_ctx.clock_source) {
	case RCC_CLOCK_LSE:
		RCC -> CCIPR |= (0b11 << 18);
		break;
	case RCC_CLOCK_LSI:
		RCC -> CCIPR |= (0b01 << 18);
		break;
	default:
		status = LPTIM_ERROR_CLOCK_SOURCE;
		goto errors;
	}
	// Clear wake-up flag.
	lptim_ctx.wake_up = 0;
	// Start timer.
	LPTIM1 -> CR |= (0b1 << 1); // SNGSTRT='1'.
	// Perform delay with the selected waiting mode.
	switch (delay_mode) {
	case LPTIM_DELAY_MODE_ACTIVE:
		// Active loop.
		while (((LPTIM1 -> ISR) & (0b1 << 1)) == 0);
		break;
	case LPTIM_DELAY_MODE_SLEEP:
		// Enable interrupt.
		NVIC_enable_interrupt(NVIC_INTERRUPT_LPTIM1, NVIC_PRIORITY_LPTIM1);
		// Enter sleep mode.
		while (lptim_ctx.wake_up == 0) {
			PWR_enter_sleep_mode();
		}
		// Disable interrupt.
		NVIC_disable_interrupt(NVIC_INTERRUPT_LPTIM1);
		break;
	case LPTIM_DELAY_MODE_STOP:
		// Enable interrupt.
		NVIC_enable_interrupt(NVIC_INTERRUPT_LPTIM1, NVIC_PRIORITY_LPTIM1);
		// Enter stop mode.
		while (lptim_ctx.wake_up == 0) {
			PWR_enter_stop_mode();
		}
		// Disable interrupt.
		NVIC_disable_interrupt(NVIC_INTERRUPT_LPTIM1);
		break;
	default:
		status = LPTIM_ERROR_DELAY_MODE;
		goto errors;
	}
errors:
	// Reset peripheral.
	RCC -> APB1RSTR |= (0b1 << 31);
	for (loop_count=0 ; loop_count<100 ; loop_count++);
	RCC -> APB1RSTR &= ~(0b1 << 31);
	// Disable peripheral clock.
	RCC -> APB1ENR &= ~(0b1 << 31); // LPTIM1EN='0'.
	// Force APB clock at the end of delay.
	RCC -> CCIPR &= ~(0b11 << 18); // LPTIM1SEL='00'.
	return status;
}
