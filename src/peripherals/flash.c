/*
 * flash.c
 *
 *  Created on: 31 march 2019
 *      Author: Ludo
 */

#include "flash.h"

#include "flash_reg.h"

/*** FLASH local macros ***/

#define FLASH_WAIT_STATES_MAX	1
#define FLASH_TIMEOUT_COUNT		100000

/*** FLASH functions ***/

/* SET FLASH LATENCY.
 * @param wait_states:	Number of wait states.
 * @return status:		Function execution status.
 */
FLASH_status_t FLASH_set_latency(unsigned char wait_states) {
	// Local variables.
	FLASH_status_t status = FLASH_SUCCESS;
	unsigned int loop_count = 0;
	// Check parameter.
	if (wait_states > FLASH_WAIT_STATES_MAX) {
		status = FLASH_ERROR_LATENCY;
		goto errors;
	}
	// Configure number of wait states.
	FLASH -> ACR &= ~(0b1 << 0); // Reset bit.
	FLASH -> ACR |= ((wait_states & 0b1) << 0); // Set latency.
	// Wait until configuration is done.
	while (((FLASH -> ACR) & (0b1 << 0)) != ((wait_states & 0b1) << 0)) {
		loop_count++;
		if (loop_count > FLASH_TIMEOUT_COUNT) {
			status = FLASH_ERROR_TIMEOUT;
			goto errors;
		}
	}
errors:
	return status;
}
