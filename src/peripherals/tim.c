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
#include "usart.h"
#include "wind.h"

/*** TIM functions ***/

/* CONFIGURE TIM21 TO OVERFLOW EVERY SECOND.
 * @param:	None.
 * @return:	None.
 */
void TIM21_Init(void) {

	/* Enable peripheral clock */
	RCC -> APB2ENR |= (0b1 << 2); // TIM21EN='1'.

	/* Reset timer before configuration */
	TIM21 -> CR1 &= ~(0b1 << 0); // Disable TIM21 (CEN='0').
	TIM21 -> CNT &= 0xFFFF0000; // Reset counter.
	TIM21 -> SR &= 0xFFFFF9B8; // Clear all flags.

	/* Configure TIM21 as master to count milliseconds and overflow every seconds */
	TIM21 -> PSC = RCC_GetSysclkKhz(); // Timer is clocked by SYSCLK (see RCC_Init() function). SYSCLK_KHZ-1 ?
	TIM21 -> ARR = 1000;
	TIM21 -> CR2 &= ~(0b111 << 4); // Reset bits 4-6.
	TIM21 -> CR2 |= (0b010 << 4); // Generate trigger on update event (MMS='010').

	/* Generate event to update registers */
	TIM21 -> EGR |= (0b1 << 0); // UG='1'.
}

/* ENABLE TIM21 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM21_Start(void) {

	/* Enable peripheral clock */
	RCC -> APB2ENR |= (0b1 << 2); // TIM21EN='1'.

	/* Enable TIM21 peripheral */
	TIM21 -> SR &= ~(0b1 << 0); // Clear flag (UIF='0').
	TIM21 -> CR1 |= (0b1 << 0); // Enable TIM21 (CEN='1').
}

/* STOP TIM21 COUNTER.
 * @param:	None.
 * @return:	None.
 */
void TIM21_Stop(void) {

	/* Stop TIM21 */
	TIM21 -> CR1 &= ~(0b1 << 0); // CEN='0'.
}

/* DISABLE TIM21 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM21_Disable(void) {

	/* Disable TIM21 peripheral */
	TIM21 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	TIM21 -> CNT = 0;
	RCC -> APB2ENR &= ~(0b1 << 2); // TIM21EN='0'.
}

/* CONFIGURE TIM22 TO COUNT SECONDS SINCE START-UP.
 * @param:	None.
 * @return:	None.
 */
void TIM22_Init(void) {

	/* Enable peripheral clock */
	RCC -> APB2ENR |= (0b1 << 5); // TIM22EN='1'.

	/* Reset timer before configuration */
	TIM22 -> CR1 &= ~(0b1 << 0); // Disable TIM22 (CEN='0').
	TIM22 -> CNT &= 0xFFFF0000; // Reset counter.
	TIM22 -> SR &= 0xFFFFF9B8; // Clear all flags.

	/* Configure TIM22 as slave to count seconds */
	TIM22 -> SMCR &= ~(0b111 << 4); // TS = '000' to select ITR0 = TIM1 as trigger input.
	TIM22 -> SMCR |= (0b111 << 0); // SMS = '111' to enable slave mode with external clock.

	/* Generate event to update registers */
	TIM22 -> EGR |= (0b1 << 0); // UG='1'.
}

/* ENABLE TIM22 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM22_Start(void) {

	/* Enable peripheral clock */
	RCC -> APB2ENR |= (0b1 << 5); // TIM22EN='1'.

	/* Enable TIM22 peripheral */
	TIM22 -> CR1 |= (0b1 << 0); // Enable TIM22 (CEN='1').
}

/* STOP TIM22 COUNTER.
 * @param:	None.
 * @return:	None.
 */
void TIM22_Stop(void) {

	/* Disable TIM22 peripheral */
	TIM22 -> CR1 &= ~(0b1 << 0); // CEN='0'.
}

/* DISABLE TIM22 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM22_Disable(void) {

	/* Disable TIM22 peripheral */
	TIM22 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	TIM22 -> CNT = 0;
	RCC -> APB2ENR &= ~(0b1 << 5); // TIM22EN='0'.
}

/* RETURNS THE NUMBER OF SECONDS ELLAPSED SINCE START-UP.
 * @param:	None.
 * @return:	Number of seconds ellapsed since start-up.
 */
volatile unsigned int TIM22_GetSeconds(void) {
	return (TIM22 -> CNT);
}

/* CONFIGURE TIM2 FOR WIND PHASE SHIFT MEASURE OR SIGFOX BPSK MODULATION.
 * @param mode:		Timer mode (see Timer2_Mode enumeration in tim.h).
 * @param timings:	Events timings given as [ARR, CCR1, CCR2, CCR3, CCR4].
 * @return:			None.
 */
void TIM2_Init(TIM2_Mode mode, unsigned short timings[TIM2_TIMINGS_ARRAY_LENGTH]) {

	/* Enable peripheral clock */
	RCC -> APB1ENR |= (0b1 << 0); // TIM2EN='1'.

	/* Reset timer before configuration */
	TIM2 -> CR1 &= 0xFFFFFE00; // Disable TIM2 (CEN='0').
	TIM2 -> CNT &= 0xFFFF0000; // Reset counter.
	TIM2 -> CR1 |= (0b1 << 2); // UIF set only on counter overflow (URS='1').

#if (defined CM || defined ATM)
	/* ocal variables */
	unsigned int arr_value = 0;
	unsigned int psc_value = 0;
#endif

	switch (mode) {

#if (defined CM || defined ATM)
	case TIM2_MODE_WIND:
		/* Configure TIM2 to overflow every (WIND_SPEED_MEASUREMENT_PERIOD_SECONDS + 1) seconds */
		arr_value = 0xFFFF; // Maximum overflow value for the desired period (to optimize "dynamic" = accuracy).
		TIM2 -> ARR &= 0xFFFF0000; // Reset all bits.
		TIM2 -> ARR |= arr_value;
		// PSC = (desired_period * timer_input_clock) / (ARR).
		psc_value = ((WIND_SPEED_MEASUREMENT_PERIOD_SECONDS + 1) * (RCC_GetSysclkKhz() * 1000)) / (arr_value);
		if (psc_value > 0xFFFF) {
			psc_value = 0xFFFF;
		}
		TIM2 -> PSC &= 0xFFFF0000; // Reset all bits.
		TIM2 -> PSC |= psc_value; // Timer is clocked by SYSCLK (see RCC_Init() function).
		break;
#endif

	case TIM2_MODE_SIGFOX:
		/* Configure TIM2 to overflow every timing[0] microseconds */
		TIM2 -> PSC = (RCC_GetSysclkKhz() / 1000) - 1; // Timer input clock = SYSCLK / (PSC + 1) = 1MHz.
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
		break;

	default:
		break;
	}

	/* Disable interrupt by default */
	NVIC_DisableInterrupt(IT_TIM2);
}

/* ENABLE TIM2 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM2_Enable(void) {

	/* Enable TIM2 peripheral */
	RCC -> APB1ENR |= (0b1 << 0); // TIM2EN='1'.
}

/* DISABLE TIM2 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM2_Disable(void) {

	/* Disable TIM2 peripheral */
	NVIC_DisableInterrupt(IT_TIM2);
	TIM2 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	TIM2 -> CNT = 0;
	RCC -> APB1ENR &= ~(0b1 << 0); // TIM2EN='0'.
}

/* START TIMER2.
 * @param:	None.
 * @return:	None.
 */
void TIM2_Start(void) {

	/* Reset and start counter */
	TIM2 -> CNT &= 0xFFFF0000;
	TIM2 -> CR1 |= (0b1 << 0); // CEN='1'.
}

/* STOP TIMER2.
 * @param:	None.
 * @return:	None.
 */
void TIM2_Stop(void) {

	/* Stop counter */
	TIM2 -> CR1 &= ~(0b1 << 0); // CEN='0'.
}

/* RETURN CURRENT TIM2 COUNTER VALUE.
 * @param:	None.
 * @return:	Current TIM2 counter value.
 */
volatile unsigned int TIM2_GetCounter(void) {
	return (TIM2 -> CNT);
}
