/*
 * lptim.c
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludo
 */

#include "lptim.h"

#include "lptim_reg.h"
#include "rcc.h"
#include "rcc_reg.h"

/*** LPTIM functions ***/

/* INIT LPTIM FOR DELAY OPERATION.
 * @param:	None.
 * @return:	None.
 */
void LPTIM1_Init(void) {

	/* Enable peripheral clock */
	RCC -> CCIPR &= (0b11 << 18); // LPTIMSEL='00' (APB clock selected = 16MHz).
	RCC -> APB1ENR |= (0b1 << 31); // LPTIM1EN='1'.

	/* Reset timer before configuration */
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0').
	LPTIM1 -> CNT = 0; // Reset counter.

	/* Configure peripheral */
	LPTIM1 -> CFGR = 0; // Reset all bits, PRESC='000' (1).
	LPTIM1 -> IER = 0; // None interrupt.
	LPTIM1 -> CR |= (0b1 << 0); // Enable LPTIM1 (ENABLE='1'), needed to write ARR.
	LPTIM1 -> ARR = RCC_SYSCLK_KHZ; // Overflow period = 1ms.
	while (((LPTIM1 -> ISR) & (0b1 << 4)) == 0); // Wait for ARROK='1'.

	/* Disable peripheral by default */
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0').
	RCC -> APB1ENR &= ~(0b1 << 31); // LPTIM1EN='0'.
}

/* DELAY FUNCTION.
 * @param delay_ms:	Number of milliseconds to wait.
 * @return:			None.
 */
void LPTIM1_DelayMilliseconds(unsigned int delay_ms) {

	/* Enable timer */
	RCC -> APB1ENR |= (0b1 << 31); // LPTIM1EN='1'.
	LPTIM1 -> CR |= (0b1 << 0); // Enable LPTIM1 (ENABLE='1').

	/* Make as many overflows as required */
	unsigned int ms_count = 0;
	for (ms_count=0 ; ms_count<delay_ms ; ms_count++) {
		// Start counter.
		LPTIM1 -> CR |= (0b1 << 1); // SNGSTRT='1'.
		while (((LPTIM1 -> ISR) & (0b1 << 1)) == 0); // Wait for counter to reach autoreload value (ARRM='1').
		LPTIM1 -> ICR |= (0b1 << 1); // Clear ARRM flag (ARRMCF='1').
		LPTIM1 -> CNT = 0;
	}

	/* Disable timer */
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0').
	RCC -> APB1ENR &= ~(0b1 << 31); // LPTIM1EN='0'.
}
