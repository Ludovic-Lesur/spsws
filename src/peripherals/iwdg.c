/*
 * iwdg.c
 *
 *  Created on: 11 aug 2018
 *      Author: Ludovic
 */

#include "iwdg.h"

#include "iwdg_reg.h"
#include "pwr_reg.h"
#include "rcc_reg.h"

/*** IWDG functions ***/

/* INIT AND START INDEPENDENT WATCHDOG.
 * @param:	None.
 * @return:	None.
 */
void IWDG_Init(void) {

	/* Enable LSI */
	RCC -> CSR |= (0b1 << 0); // LSION='1'.
	while (((RCC -> CSR) & (0b1 << 1)) == 0); // Wait for LSIRDY='1'.

	/* Configure peripheral */
	IWDG -> KR = 0x0000CCCC; // Enable peripheral.
	IWDG -> KR = 0x00005555; // Enable register access.
	// Prescaler=128 -> watchdog clock = 32kHz / 128 = 250Hz (4ms period).
	IWDG -> PR = (0b101 << 0); // PR='101'.
	// Set reload value.
	IWDG -> RLR = 2500; // 2500*4ms = 10s.
	// Wait for register to be updated.
	while (IWDG -> SR != 0); // Wait for WVU='0', RVU='0' and PVU='0'.
}

/* RELOAD WATCHDOG COUNTER.
 * @param:	None.
 * @return:	None.
 */
void IWDG_Reload(void) {
	// Reload counter.
	IWDG -> KR = 0x0000AAAA;
}
