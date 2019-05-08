/*
 * iwdg.c
 *
 *  Created on: 30 mars 2019
 *      Author: Ludovic
 */

#include "iwdg.h"

#include "iwdg_reg.h"

/*** IWDG functions ***/

/* INIT AND START INDEPENDENT WATCHDOG.
 * @param:	None.
 * @return:	None.
 */
void IWDG_Init(void) {
	/* Configure peripheral */
	IWDG -> KR = 0x0000CCCC; // Enable peripheral.
	IWDG -> KR = 0x00005555; // Enable register access.
	// Prescaler=256 -> watchdog clock = 38kHz / 256 = 148Hz (6.74ms period).
	IWDG -> PR |= (0b111 << 0); // PR='111'.
	// Set reload value.
	IWDG -> RLR = 1484; // 1484 * 6.74ms = 10s.
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
