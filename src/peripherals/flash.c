/*
 * flash.c
 *
 *  Created on: 31 march 2019
 *      Author: Ludovic
 */

#include "flash.h"

#include "flash_reg.h"

/*** FLASH local macros ***/

#define FLASH_TIMEOUT_COUNT		1000

/*** FLASH functions ***/

/* INIT FLASH LATENCY.
 * @param:	None.
 * @return:	None.
 */
void FLASH_Init(void) {
	// Add 1 wait state.
	FLASH -> ACR |= (0b1 << 0); // LATENCY='1'.
	// Wait until configuration is done.
	unsigned int count = 0;
	while ((((FLASH -> ACR) & (0b1 << 0)) == 0) && (count < FLASH_TIMEOUT_COUNT)) {
		count++;
	}
}
