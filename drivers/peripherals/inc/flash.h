/*
 * flash.h
 *
 *  Created on: 31 mar. 2019
 *      Author: Ludo
 */

#ifndef __FLASH_H__
#define __FLASH_H__

#include "types.h"

/*** FLASH structures ***/

/*!******************************************************************
 * \enum FLASH_status_t
 * \brief FLASH driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	FLASH_SUCCESS = 0,
	FLASH_ERROR_LATENCY,
	FLASH_ERROR_TIMEOUT,
	// Last base value.
	FLASH_ERROR_BASE_LAST = 0x0100
} FLASH_status_t;

/*** FLASH functions ***/

/*!******************************************************************
 * \fn FLASH_status_t FLASH_set_latency(uint8_t wait_states)
 * \brief Set FLASH latency.
 * \param[in]  	wait_states: Number of wait states to set.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
FLASH_status_t FLASH_set_latency(uint8_t wait_states);

/*******************************************************************/
#define FLASH_exit_error(error_base) { if (flash_status != FLASH_SUCCESS) { status = (error_base + flash_status); goto errors; } }

/*******************************************************************/
#define FLASH_stack_error(void) { if (flash_status != FLASH_SUCCESS) { ERROR_stack_add(ERROR_BASE_FLASH + flash_status); } }

/*******************************************************************/
#define FLASH_stack_exit_error(error_code) { if (flash_status != FLASH_SUCCESS) { ERROR_stack_add(ERROR_BASE_FLASH + flash_status); status = error_code; goto errors; } }

#endif /* __FLASH_H__ */
