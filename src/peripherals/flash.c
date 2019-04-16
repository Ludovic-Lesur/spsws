/*
 * flash.c
 *
 *  Created on: 31 march 2019
 *      Author: Ludovic
 */

#include "flash.h"

#include "flash_reg.h"

/*** FLASH functions ***/

/* INIT FLASH LATENCY.
 * @param:	None.
 * @return:	None.
 */
void FLASH_Init(void) {
	// Add 1 wait state.
	FLASH -> ACR |= (0b1 << 0); // LATENCY='1'.
}
