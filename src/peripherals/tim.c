/*
 * tim.c
 *
 *  Created on: 4 may 2018
 *      Author: Ludovic
 */

#include "tim.h"

#include "rcc.h"
#include "rcc_reg.h"
#include "tim_reg.h"

/*** TIM functions ***/

/* CONFIGURE TIM21 and TIM22 TO COUNT MILLISECONDS AND SECONDS SINCE START-UP.
 * @param:	None.
 * @return:	None.
 */
void TIM_TimeInit(void) {

	/* Enable peripherals clock */
	RCC -> APB2ENR |= (0b1001 << 2); // TIM21EN='1' and TIM22EN='1'.

	/* Reset timers before configure them */
	TIM21 -> CR1 &= ~(0b1 << 0); // Disable TIM21 (CEN='0').
	TIM21 -> CNT = 0; // Reset counter.
	TIM22 -> CR1 &= ~(0b1 << 0); // Disable TIM22 (CEN='0').
	TIM22 -> CNT = 0; // Reset counter.

	/* Configure TIM21 as master to count milliseconds and overflow every seconds */
	TIM21 -> PSC = SYSCLK_KHZ; // Timer is clocked by SYSCLK (see RCC_Init() function). SYSCLK_KHZ-1 ?
	TIM21 -> ARR = 1000; // 999 ?
	TIM21 -> CR2 &= ~(0b111 << 4); // Reset bits 4-6.
	TIM21 -> CR2 |= (0b010 << 4); // Generate trigger on update event (MMS='010').

	/* Configure TIM22 as slave to count seconds */
	TIM22 -> SMCR &= ~(0b111 << 4); // TS = '000' to select ITR0 = TIM1 as trigger input.
	TIM22 -> SMCR |= (0b111 << 0); // SMS = '111' to enable slave mode with external clock.

	/* Generate event to update registers */
	TIM21 -> EGR |= (0b1 << 0); // UG='1'.
	TIM22 -> EGR |= (0b1 << 0); // UG='1'.

	/* Start timers */
	TIM22 -> CR1 |= (0b1 << 0); // Enable TIM22 (CEN='1').
	TIM21 -> CR1 |= (0b1 << 0); // Enable TIM21 (CEN='1').
}

/* RETURNS THE NUMBER OF SECONDS ELLAPSED SINCE START-UP.
 * @param:	None.
 * @return:	Number of seconds ellapsed since start-up.
 */
unsigned int TIM_TimeGetSeconds(void) {
	return (TIM22 -> CNT);
}

/* RETURNS THE NUMBER OF MILLISECONDS ELLAPSED SINCE START-UP.
 * @param:	None.
 * @return:	Number of milliseconds ellapsed since start-up.
 */
unsigned int TIM_TimeGetMilliseconds(void) {
	return (TIM_TimeGetSeconds()*1000 + (TIM21 -> CNT));
}

/* DELAY FUNCTION.
 * @param msToWait:	Number of milliseconds to wait.
 * @return:			None.
 */
void TIM_TimeWaitMilliseconds(unsigned int ms_to_wait) {
	unsigned int start_ms = TIM_TimeGetMilliseconds();
	while (TIM_TimeGetMilliseconds() < (start_ms + ms_to_wait));
}
