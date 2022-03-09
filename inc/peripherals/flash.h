/*
 * flash.h
 *
 *  Created on: 31 march 2019
 *      Author: Ludovic
 */

#ifndef FLASH_H
#define FLASH_H

/*** FLASH structures ***/

typedef enum {
	FLASH_SUCCESS = 0,
	FLASH_ERROR_LATENCY,
	FLASH_ERROR_TIMEOUT,
	FLASH_ERROR_BASE_LAST = 0x0100
} FLASH_status_t;

/*** FLASH functions ***/

FLASH_status_t FLASH_set_latency(unsigned char wait_states);

#define FLASH_status_check(error_base) { if (flash_status != FLASH_SUCCESS) { status = error_base + flash_status; goto errors; }}

#endif /* FLASH_H */
