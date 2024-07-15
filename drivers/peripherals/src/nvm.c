/*
 * nvm.c
 *
 *  Created on: 19 jun. 2018
 *      Author: Ludo
 */

#include "nvm.h"

#include "flash_reg.h"
#include "rcc_reg.h"
#include "types.h"

/*** NVM local macros ***/

#define NVM_TIMEOUT_COUNT	1000000

/*** NVM local functions ***/

/*******************************************************************/
static NVM_status_t _NVM_check_busy(NVM_status_t timeout_error_code) {
	// Local variables.
	NVM_status_t status = NVM_SUCCESS;
	uint32_t loop_count = 0;
	// Check no write/erase operation is running.
	while (((FLASH -> SR) & (0b1 << 0)) != 0) {
		// Wait till BSY='1' or timeout.
		loop_count++;
		if (loop_count > NVM_TIMEOUT_COUNT) {
			status = timeout_error_code;
			goto errors;
		}
	}
errors:
	return status;
}

/*******************************************************************/
static NVM_status_t __attribute__((optimize("-O0"))) _NVM_unlock(void) {
	// Local variables.
	NVM_status_t status = NVM_SUCCESS;
	// Check memory is ready.
	status = _NVM_check_busy(NVM_ERROR_UNLOCK);
	if (status != NVM_SUCCESS) goto errors;
	// Check the memory is not already unlocked.
	if (((FLASH -> PECR) & (0b1 << 0)) != 0) {
		// Perform unlock sequence.
		FLASH -> PEKEYR = 0x89ABCDEF;
		FLASH -> PEKEYR = 0x02030405;
	}
errors:
	return status;
}

/*******************************************************************/
static NVM_status_t __attribute__((optimize("-O0"))) _NVM_lock(void) {
	// Local variables.
	NVM_status_t status = NVM_SUCCESS;
	// Check memory is ready.
	status = _NVM_check_busy(NVM_ERROR_LOCK);
	if (status != NVM_SUCCESS) goto errors;
	// Lock sequence.
	FLASH -> PECR |= (0b1 << 0);
errors:
	return status;
}

/*** NVM functions ***/

/*******************************************************************/
NVM_status_t __attribute__((optimize("-O0"))) NVM_read_byte(NVM_address_t address, uint8_t* data) {
	// Local variables.
	NVM_status_t status = NVM_SUCCESS;
	uint32_t absolute_address = (EEPROM_START_ADDRESS + address);
	// Check parameters.
	if (address >= EEPROM_SIZE_BYTES) {
		status = NVM_ERROR_ADDRESS;
		goto errors;
	}
	if (data == NULL) {
		status = NVM_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Enable peripheral.
	RCC -> AHBENR |= (0b1 << 8); // MIFEN='1'.
	// Check there is no pending operation.
	status = _NVM_check_busy(NVM_ERROR_READ);
	if (status != NVM_SUCCESS) goto errors;
	// Read data.
	(*data) = *((uint8_t*) (absolute_address));
errors:
	// Disable peripheral.
	RCC -> AHBENR &= ~(0b1 << 8); // MIFEN='0'.
	return status;
}

/*******************************************************************/
NVM_status_t __attribute__((optimize("-O0"))) NVM_write_byte(NVM_address_t address, uint8_t data) {
	// Local variables.
	NVM_status_t status = NVM_SUCCESS;
	uint32_t absolute_address = (EEPROM_START_ADDRESS + address);
	// Check parameters.
	if (address >= EEPROM_SIZE_BYTES) {
		status = NVM_ERROR_ADDRESS;
		goto errors;
	}
	// Enable peripheral.
	RCC -> AHBENR |= (0b1 << 8); // MIFEN='1'.
	// Unlock memory.
	status = _NVM_unlock();
	if (status != NVM_SUCCESS) goto errors;
	// Write data.
	(*((uint8_t*) (absolute_address))) = data;
	// Wait the end of operation.
	status = _NVM_check_busy(NVM_ERROR_WRITE);
	if (status != NVM_SUCCESS) goto errors;
	// Lock memory.
	status = _NVM_lock();
	if (status != NVM_SUCCESS) goto errors;
	// Disable peripheral.
	RCC -> AHBENR &= ~(0b1 << 8); // MIFEN='0'.
	return status;
errors:
	// Lock memory.
	_NVM_lock();
	// Disable peripheral.
	RCC -> AHBENR &= ~(0b1 << 8); // MIFEN='0'.
	return status;
}
