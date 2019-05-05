/*
 * lptim.c
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludo
 */

#include "lptim.h"

#include "lptim_reg.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "tim.h"

/*** LPTIM local macros ***/

#define LPTIM_TIMEOUT_SECONDS	3

/*** LPTIM local global variables ***/

static volatile unsigned char lptim_arrm_flag;

/*** LPTIM local functions ***/

/* LPTIM INTERRUPT HANDLER
 * @param:	None.
 * @return:	None.
 */
void LPTIM1_IRQHandler(void) {

	/* ARRM interrupt */
	if (((LPTIM1 -> ISR) & (0b1 << 1)) != 0) {
		// Update local flag.
		lptim_arrm_flag = 1;
		// Clear flag (ARRMCF='1').
		LPTIM1 -> ICR |= (0b1 << 1);
	}
}

/*** LPTIM functions ***/

/* INIT LPTIM FOR DELAY OPERATION.
 * @param lptim1_use_lsi:	Use APB as clock source if 0, LSI otherwise.
 * @return:					None.
 */
void LPTIM1_Init(unsigned char lptim1_use_lsi) {

	/* Disable peripheral */
	RCC -> APB1ENR &= ~(0b1 << 31); // LPTIM1EN='0'.

	/* Enable peripheral clock */
	RCC -> CCIPR &= ~(0b11 << 18); // LPTIMSEL='00' (APB clock selected = 16MHz).
	if (lptim1_use_lsi != 0) {
		RCC -> CCIPR |= (0b01 << 18); // LPTIMSEL='01' (LSI clock selected).
	}
	RCC -> APB1ENR |= (0b1 << 31); // LPTIM1EN='1'.

	/* Configure peripheral */
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0'), needed to write CFGR.
	LPTIM1 -> CFGR |= (0b1 << 19); // Enable timeout.
	LPTIM1 -> IER |= (0b1 << 1); // Enable ARRM interrupt (ARRMIE='1').
	LPTIM1 -> CNT &= 0xFFFF0000; // Reset counter.
	LPTIM1 -> CR |= (0b1 << 0); // Enable LPTIM1 (ENABLE='1'), needed to write ARR.
	if (lptim1_use_lsi != 0) {
		LPTIM1 -> ARR = 0xFFFF; // Maximum overflow period.
	}
	else {
		LPTIM1 -> ARR = RCC_SYSCLK_KHZ; // Overflow period = 1ms.
	}
	unsigned int loop_start_time = TIM22_GetSeconds();
	while (((LPTIM1 -> ISR) & (0b1 << 4)) == 0) {
		// Wait for ARROK='1' or timeout.
		if (TIM22_GetSeconds() > (loop_start_time + LPTIM_TIMEOUT_SECONDS)) break;
	}

	/* Clear all flags */
	lptim_arrm_flag = 0;
	LPTIM1 -> ICR |= (0b1111111 << 0);

	/* Disable peripheral by default */
	NVIC_DisableInterrupt(IT_LPTIM1);
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0').
}

/* START LPTIM1.
 * @param:	None.
 * @return:	None.
 */
void LPTIM1_Start(void) {

	/* Enable timer */
	LPTIM1 -> CR |= (0b1 << 0); // Enable LPTIM1 (ENABLE='1').
	NVIC_EnableInterrupt(IT_LPTIM1);
	LPTIM1 -> CR |= (0b1 << 1); // SNGSTRT='1'.
}

/* STOP LPTIM1.
 * @param:	None.
 * @return:	None.
 */
void LPTIM1_Stop(void) {

	/* Stop timer */
	NVIC_DisableInterrupt(IT_LPTIM1);
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0').
}

/* DISABLE LPTIM1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void LPTIM1_Disable(void) {

	/* Disable timer */
	NVIC_DisableInterrupt(IT_LPTIM1);
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0').
	RCC -> APB1ENR &= ~(0b1 << 31); // LPTIM1EN='0'.
}

/* DELAY FUNCTION.
 * @param delay_ms:	Number of milliseconds to wait.
 * @return:			None.
 */
void LPTIM1_DelayMilliseconds(unsigned int delay_ms) {

	/* Enable timer */
	LPTIM1 -> CR |= (0b1 << 0); // Enable LPTIM1 (ENABLE='1').
	NVIC_EnableInterrupt(IT_LPTIM1);

	/* Make as many overflows as required */
	unsigned int ms_count = 0;
	unsigned int loop_start_time = 0;
	unsigned char lptim_default = 0;
	for (ms_count=0 ; ms_count<delay_ms ; ms_count++) {
		// Start counter.
		loop_start_time = TIM22_GetSeconds();
		LPTIM1 -> CNT &= 0xFFFF0000;
		LPTIM1 -> CR |= (0b1 << 1); // SNGSTRT='1'.
		while (lptim_arrm_flag == 0) {
			if (TIM22_GetSeconds() > (loop_start_time + LPTIM_TIMEOUT_SECONDS)) {
				lptim_default = 1;
				break;
			}
		}
		lptim_arrm_flag = 0;
		if (lptim_default != 0) {
			break;
		}
	}

	/* Disable timer */
	NVIC_DisableInterrupt(IT_LPTIM1);
	LPTIM1 -> CR &= ~(0b1 << 0); // Disable LPTIM1 (ENABLE='0').
}
