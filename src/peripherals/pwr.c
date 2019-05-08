/*
 * pwr.c
 *
 *  Created on: 5 may 2018
 *      Author: Ludo
 */

#include "pwr.h"

#include "mapping.h"
#include "pwr_reg.h"
#include "rcc_reg.h"
#include "scb_reg.h"

/*** PWR functions ***/

/* FUNCTION TO ENTER SLEEP MODE.
 * @param:	None.
 * @return:	None.
 */
void PWR_EnterSleepMode(void) {

	/* Enable power interface clock */
	RCC -> APB1ENR |= (0b1 << 28); // PWREN='1'.

	/* Enter low power sleep mode */
	SCB -> SCR &= ~(0b1 << 1); // Do not return in low power sleep mode after wake-up (SLEEPONEXIT='0').
	SCB -> SCR &= ~(0b1 << 2); // SLEEPDEEP='0'.
	__asm volatile ("wfi"); // Wait For Interrupt core instruction.
}

/* FUNCTION TO ENTER STANDBY MODE.
 * @param:	None.
 * @return:	None.
 */
void PWR_EnterStandbyMode(void) {

	/* Enable power interface clock */
	RCC -> APB1ENR |= (0b1 << 28); // PWREN='1'.

#ifdef USE_HWT
	/* Configure WKUP pin */
	PWR -> CSR |= (0b1 << 8); // Use WKUP1 pin (PA0) rising edge to exit standby mode (EWUP1='1').
#endif

	/* Clear WUF flag */
	PWR -> CR |= (0b1 << 2); // CWUF='1'.

	/* Switch internal voltage reference off in low power mode */
	PWR -> CR |= (0b1 << 9); // ULP='1'.

	/* Enter standy mode when CPU enters deepsleep */
	PWR -> CR |= (0b1 << 1); // PDDS='1'.

	/* Enter standby mode */
	SCB -> SCR &= ~(0b1 << 1); // Do not return in standby after wake-up (SLEEPONEXIT='0').
	SCB -> SCR |= (0b1 << 2); // SLEEPDEEP='1'.
	__asm volatile ("wfi"); // Wait For Interrupt core instruction.
}
