/*
 * flash.c
 *
 *  Created on: 31 march 2019
 *      Author: Ludovic
 */

#include "flash.h"

#include "flash_reg.h"

/*** FLASH local macros ***/

#define FLASH_TIMEOUT_COUNT		10000

/*** FLASH functions ***/

/* SET FLASH LATENCY.
 * @param wait_states:	Number of wait states.
 * @return:				None.
 */
void FLASH_SetLatency(unsigned char wait_states) {
	// Configure number of wait states.
	FLASH -> ACR &= ~(0b1 << 0); // Reset bit.
	FLASH -> ACR |= ((wait_states & 0b1) << 0); // Set latency.
	// Wait until configuration is done.
	unsigned int count = 0;
	while ((((FLASH -> ACR) & (0b1 << 0)) != ((wait_states & 0b1) << 0)) && (count < FLASH_TIMEOUT_COUNT)) {
		count++;
	}
}
