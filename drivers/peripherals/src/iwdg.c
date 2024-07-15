/*
 * iwdg.c
 *
 *  Created on: 30 mar. 2018
 *      Author: Ludo
 */

#include "iwdg.h"

#include "iwdg_reg.h"
#include "types.h"

/*** IWDG local macros ***/

#define IWDG_TIMEOUT_COUNT		100000

/*** IWDG functions ***/

/*******************************************************************/
IWDG_status_t IWDG_init(void) {
	// Local variables.
	IWDG_status_t status = IWDG_SUCCESS;
	uint32_t loop_count = 0;
	// Configure peripheral.
	IWDG -> KR = 0x0000CCCC; // Enable peripheral.
	IWDG -> KR = 0x00005555; // Enable register access.
	// Prescaler=256 -> watchdog clock = 38kHz / 256 = 148Hz.
	IWDG -> PR |= (0b111 << 0); // PR='111'.
	// Set reload value.
	IWDG -> RLR |= 0x00000FFF; // 4095 * (prescaler / LSI) = 27s.
	// Wait for WVU='0', RVU='0' and PVU='0' or timeout.
	while (IWDG -> SR != 0) {
		loop_count++;
		if (loop_count > IWDG_TIMEOUT_COUNT) {
			status = IWDG_ERROR_TIMEOUT;
			break;
		}
	}
	return status;
}

/*******************************************************************/
void IWDG_reload(void) {
	// Reload counter.
	IWDG -> KR = 0x0000AAAA;
}
