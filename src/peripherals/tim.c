/*
 * tim.c
 *
 *  Created on: 4 may 2018
 *      Author: Ludo
 */

#include "tim.h"

#include "mode.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "tim_reg.h"
#include "wind.h"
#include "types.h"

/*** TIM local macros ***/

#define TIM_TIMEOUT_COUNT	1000000

/*** TIM local global variables ***/

static volatile uint8_t tim21_flag = 0;

/*** TIM local functions ***/

/* TIM21 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) TIM21_IRQHandler(void) {
	// TI1 interrupt.
	if (((TIM21 -> SR) & (0b1 << 1)) != 0) {
		// Update flags.
		if (((TIM21 -> DIER) & (0b1 << 1)) != 0) {
			tim21_flag = 1;
		}
		TIM21 -> SR &= ~(0b1 << 1);
	}
}

/*** TIM functions ***/

/* CONFIGURE TIM21 FOR LSI FREQUENCY MEASUREMENT.
 * @param:	None.
 * @return:	None.
 */
void TIM21_init(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 2); // TIM21EN='1'.
	// Configure timer.
	// Channel input on TI1.
	// Capture done every 8 edges.
	// CH1 mapped on LSI.
	TIM21 -> CCMR1 |= (0b01 << 0) | (0b11 << 2);
	TIM21 -> OR |= (0b101 << 2);
	// Enable interrupt.
	TIM21 -> DIER |= (0b1 << 1); // CC1IE='1'.
	// Generate event to update registers.
	TIM21 -> EGR |= (0b1 << 0); // UG='1'.
}

/* MEASURE LSI CLOCK FREQUENCY WITH TIM21 CH1.
 * @param lsi_frequency_hz:		Pointer that will contain measured LSI frequency in Hz.
 * @return status:				Function execution status.
 */
TIM_status_t TIM21_get_lsi_frequency(uint32_t* lsi_frequency_hz) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	uint8_t tim21_interrupt_count = 0;
	uint32_t tim21_ccr1_edge1 = 0;
	uint32_t tim21_ccr1_edge8 = 0;
	uint32_t loop_count = 0;
	// Reset counter.
	TIM21 -> CNT = 0;
	TIM21 -> CCR1 = 0;
	// Enable interrupt.
	TIM21 -> SR &= 0xFFFFF9B8; // Clear all flags.
	NVIC_enable_interrupt(NVIC_IT_TIM21);
	// Enable TIM21 peripheral.
	TIM21 -> CR1 |= (0b1 << 0); // CEN='1'.
	TIM21 -> CCER |= (0b1 << 0); // CC1E='1'.
	// Wait for 2 captures.
	while (tim21_interrupt_count < 2) {
		// Wait for interrupt.
		tim21_flag = 0;
		loop_count = 0;
		while (tim21_flag == 0) {
			loop_count++;
			if (loop_count > TIM_TIMEOUT_COUNT) {
				status = TIM_ERROR_INTERRUPT_TIMEOUT;
				goto errors;
			}
		}
		tim21_interrupt_count++;
		if (tim21_interrupt_count == 1) {
			tim21_ccr1_edge1 = (TIM21 -> CCR1);
		}
		else {
			tim21_ccr1_edge8 = (TIM21 -> CCR1);
		}
	}
	// Compute LSI frequency.
	(*lsi_frequency_hz) = (8 * RCC_HSI_FREQUENCY_KHZ * 1000) / (tim21_ccr1_edge8 - tim21_ccr1_edge1);
errors:
	// Disable interrupt.
	NVIC_disable_interrupt(NVIC_IT_TIM21);
	// Stop counter.
	TIM21 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	TIM21 -> CCER &= ~(0b1 << 0); // CC1E='0'.
	return status;
}

/* DISABLE TIM21 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM21_disable(void) {
	// Disable TIM21 peripheral.
	TIM21 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	RCC -> APB2ENR &= ~(0b1 << 2); // TIM21EN='0'.
}

/* CONFIGURE TIM2 FOR SIGFOX BPSK MODULATION.
 * @param timings:	Events timings given as [ARR, CCR1, CCR2, CCR3, CCR4].
 * @return:			None.
 */
void TIM2_init(uint16_t timings[TIM2_TIMINGS_ARRAY_LENGTH]) {
	// Enable peripheral clock.
	RCC -> APB1ENR |= (0b1 << 0); // TIM2EN='1'.
	// Configure TIM2 to overflow every timing[0] microseconds.
	TIM2 -> CR1 |= (0b1 << 2); // UIF set only on counter overflow (URS='1').
	TIM2 -> PSC = (RCC_get_sysclk_khz() / 1000) - 1; // Timer input clock = SYSCLK / (PSC + 1) = 1MHz.
	TIM2 -> ARR = timings[TIM2_TIMINGS_ARRAY_ARR_IDX];
	// Configure events timestamps.
	TIM2 -> CCR1 = timings[TIM2_TIMINGS_ARRAY_CCR1_IDX];
	TIM2 -> CCR2 = timings[TIM2_TIMINGS_ARRAY_CCR2_IDX];
	TIM2 -> CCR3 = timings[TIM2_TIMINGS_ARRAY_CCR3_IDX];
	TIM2 -> CCR4 = timings[TIM2_TIMINGS_ARRAY_CCR4_IDX];
	// Enable channels (OCxm='001').
	TIM2 -> CCMR1 |= (0b001 << 12) | (0b011 << 4);
	TIM2 -> CCMR2 |= (0b001 << 12) | (0b011 << 4);
	// Generate event to update registers.
	TIM21 -> EGR |= (0b1 << 0); // UG='1'.
	// Enable update and CCRx interrupts.
	TIM2 -> DIER |= (0b11111 << 0);
	// Disable interrupt by default.
	NVIC_set_priority(NVIC_IT_TIM2, 0);
	NVIC_disable_interrupt(NVIC_IT_TIM2);
}

/* START TIMER2.
 * @param:	None.
 * @return:	None.
 */
void TIM2_start(void) {
	// Enable interrupt.
	NVIC_enable_interrupt(NVIC_IT_TIM2);
	// Reset and start counter.
	TIM2 -> CNT &= 0xFFFF0000;
	TIM2 -> SR &= 0xFFFFFFE0;
	TIM2 -> CR1 |= (0b1 << 0); // CEN='1'.
}

/* STOP TIMER2.
 * @param:	None.
 * @return:	None.
 */
void TIM2_stop(void) {
	// Stop counter.
	TIM2 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	// Disable interrupt.
	NVIC_disable_interrupt(NVIC_IT_TIM2);
}

/* RETURN CURRENT TIM2 COUNTER VALUE.
 * @param:	None.
 * @return:	Current TIM2 counter value.
 */
volatile uint32_t TIM2_get_counter(void) {
	return (TIM2 -> CNT);
}
