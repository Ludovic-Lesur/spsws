/*
 * lptim.c
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludo
 */

#include "lptim.h"

#include "exti.h"
#include "lptim_reg.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "wind.h"

/*** LPTIM local macros ***/

#define LPTIM_TIMEOUT_COUNT		1000000
#define LPTIM_DELAY_MS_MIN		1
#define LPTIM_DELAY_MS_MAX		55000

/*** LPTIM local global variables ***/

static unsigned int lptim_clock_frequency_hz = 0;
static volatile unsigned char lptim_wake_up = 0;

/*** LPTIM local functions ***/

/* LPTIM INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) LPTIM1_IRQHandler(void) {
	// Check flag.
	if (((LPTIM1 -> ISR) & (0b1 << 1)) != 0) {
		// Update flags.
		if (((LPTIM1 -> IER) & (0b1 << 1)) != 0) {
			lptim_wake_up = 1;
		}
		LPTIM1 -> ICR |= (0b1 << 1);
	}
}

/* WRITE ARR REGISTER.
 * @param arr_value:	ARR register value to write.
 * @return:				None.
 */
static void LPTIM1_WriteArr(unsigned int arr_value) {
	unsigned int loop_count = 0;
	// Reset bits.
	LPTIM1 -> ICR |= (0b1 << 4);
	LPTIM1 -> ARR &= 0xFFFF0000;
	while (((LPTIM1 -> ISR) & (0b1 << 4)) == 0) {
		// Wait for ARROK='1' or timeout.
		loop_count++;
		if (loop_count > LPTIM_TIMEOUT_COUNT) break;
	}
	// Write new value.
	LPTIM1 -> ICR |= (0b1 << 4);
	LPTIM1 -> ARR |= arr_value;
	loop_count = 0;
	while (((LPTIM1 -> ISR) & (0b1 << 4)) == 0) {
		// Wait for ARROK='1' or timeout.
		loop_count++;
		if (loop_count > LPTIM_TIMEOUT_COUNT) break;
	}
}

/*** LPTIM functions ***/

/* INIT LPTIM FOR DELAY OPERATION.
 * @param lsi_freq_hz:	Effective LSI oscillator frequency.
 * @return:				None.
 */
void LPTIM1_Init(unsigned int lsi_freq_hz) {
	// Disable peripheral.
	RCC -> APB1ENR &= ~(0b1 << 31); // LPTIM1EN='0'.
	// Enable peripheral clock.
	RCC -> CCIPR &= ~(0b11 << 18); // Reset bits 18-19.
	RCC -> CCIPR |= (0b01 << 18); // LPTIMSEL='01' (LSI clock selected).
	lptim_clock_frequency_hz = (lsi_freq_hz >> 5);
	RCC -> APB1ENR |= (0b1 << 31); // LPTIM1EN='1'.
	// Configure peripheral.
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0'), needed to write CFGR.
	LPTIM1 -> CFGR &= ~(0b1 << 0);
	LPTIM1 -> CFGR |= (0b101 << 9); // Prescaler = 32.
	LPTIM1 -> CNT &= 0xFFFF0000; // Reset counter.
	// Enable LPTIM EXTI line.
	LPTIM1 -> IER |= (0b1 << 1); // ARRMIE='1'.
	EXTI_ConfigureLine(EXTI_LINE_LPTIM1, EXTI_TRIGGER_RISING_EDGE);
	// Set interrupt priority.
	NVIC_SetPriority(NVIC_IT_LPTIM1, 2);
	// Clear all flags.
	LPTIM1 -> ICR |= (0b1111111 << 0);
}

/* ENABLE LPTIM1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void LPTIM1_Enable(void) {
	// Enable timer clock.
	RCC -> APB1ENR |= (0b1 << 31); // LPTIM1EN='1'.
}

/* DISABLE LPTIM1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void LPTIM1_Disable(void) {
	// Disable timer.
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0').
	LPTIM1 -> CNT &= 0xFFFF0000;
	// Clear all flags.
	LPTIM1 -> ICR |= (0b1111111 << 0);
	// Disable peripheral clock.
	RCC -> APB1ENR &= ~(0b1 << 31); // LPTIM1EN='0'.
}

/* DELAY FUNCTION.
 * @param delay_ms:		Number of milliseconds to wait.
 * @param stop_mode:	Enter stop mode during delay if non zero.
 * @return:				None.
 */
void LPTIM1_DelayMilliseconds(unsigned int delay_ms, unsigned char stop_mode) {
	// Clamp value if required.
	unsigned int local_delay_ms = delay_ms;
	if (local_delay_ms > LPTIM_DELAY_MS_MAX) {
		local_delay_ms = LPTIM_DELAY_MS_MAX;
	}
	if (local_delay_ms < LPTIM_DELAY_MS_MIN) {
		local_delay_ms = LPTIM_DELAY_MS_MIN;
	}
	// Enable timer.
	LPTIM1 -> CR |= (0b1 << 0); // Enable LPTIM1 (ENABLE='1').
	// Reset counter.
	LPTIM1 -> CNT &= 0xFFFF0000;
	// Compute ARR value.
	unsigned int arr = ((local_delay_ms * lptim_clock_frequency_hz) / (1000)) & 0x0000FFFF;
	LPTIM1_WriteArr(arr);
	// Start timer.
	NVIC_EnableInterrupt(NVIC_IT_LPTIM1);
	lptim_wake_up = 0;
	LPTIM1 -> CR |= (0b1 << 1); // SNGSTRT='1'.
	// Wait for interrupt.
	while (lptim_wake_up == 0) {
		if (stop_mode != 0) {
			PWR_EnterStopMode();
		}
	}
	// Disable timer.
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0').
	NVIC_DisableInterrupt(NVIC_IT_LPTIM1);
}

#ifdef WIND_VANE_ULTIMETER
/* START LPTIM COUNTER.
 * @param:	None.
 * @return:	None.
 */
void LPTIM1_Start(void) {
	// Enable timer.
	LPTIM1 -> CR |= (0b1 << 0); // Enable LPTIM1 (ENABLE='1').
	// Reset counter.
	LPTIM1 -> CNT &= 0xFFFF0000;
	// Set ARR to maximum value (unused).
	LPTIM1_WriteArr(0xFFFF);
	// Start timer.
	LPTIM1 -> CR |= (0b1 << 1); // SNGSTRT='1'.
}

/* START LPTIM COUNTER.
 * @param:	None.
 * @return:	None.
 */
void LPTIM1_Stop(void) {
	// Disable timer.
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0').
}

/* START LPTIM COUNTER.
 * @param:	None.
 * @return:	LPTIM1 counter value.
 */
unsigned int LPTIM1_GetCounter(void) {
	return (LPTIM1 -> CNT);
}
#endif
