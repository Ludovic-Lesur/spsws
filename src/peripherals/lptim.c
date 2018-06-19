/*
 * lptim.c
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludovic
 */

#include "lptim.h"
#include "lptim_reg.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"

/*** LPTIM functions ***/

void LPTIM_Init(void) {

	/* Enable peripheral clock */
	RCC -> APB1ENR |= (0b1 << 31); // LPTIMEN='1'.

	/* Configure peripheral */
	LPTIM1 -> CFGR &= ~(0b111 << 9); // Prescaler=1, input clock=SYSCLK. (PRES='000').

	/* Configure interrupts */
	LPTIM1 -> IER |= (0b1 << 1); // Enable autoreload match interrupt (ARRMIE='1').

	/* Enable peripheral */
	LPTIM1 -> CR |= (0b1 << 0); // (ENABLE='1').
}

