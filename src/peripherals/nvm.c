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
static NVM_status_t _NVM_unlock(void) {
	// Local variables.
	NVM_status_t status = NVM_SUCCESS;
	uint32_t loop_count = 0;
	// Check no write/erase operation is running.
	while (((FLASH -> SR) & (0b1 << 0)) != 0) {
		// Wait till BSY='1' or timeout.
		loop_count++;
		if (loop_count > NVM_TIMEOUT_COUNT) {
			status = NVM_ERROR_UNLOCK;
			goto errors;
		}
	}
	// Check the NVM is not already unlocked.
	if (((FLASH -> PECR) & (0b1 << 0)) != 0) {
		// Perform unlock sequence.
		FLASH -> PEKEYR = 0x89ABCDEF;
		FLASH -> PEKEYR = 0x02030405;
	}
errors:
	return status;
}

/*******************************************************************/
static NVM_status_t _NVM_lock(void) {
	// Local variables.
	NVM_status_t status = NVM_SUCCESS;
	uint32_t loop_count = 0;
	// Check no write/erase operation is running.
	while (((FLASH -> SR) & (0b1 << 0)) != 0) {
		// Wait till BSY='1' or timeout.
		loop_count++;
		if (loop_count > NVM_TIMEOUT_COUNT) {
			status = NVM_ERROR_LOCK;
			goto errors;
		}
	}
	// Lock PECR register.
	FLASH -> PECR |= (0b1 << 0); // PELOCK='1'.
errors:
	return status;
}

/*** NVM functions ***/

/*******************************************************************/
NVM_status_t NVM_read_byte(NVM_address_t address, uint8_t* data) {
	// Local variables.
	NVM_status_t status = NVM_SUCCESS;
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
	// Unlock NVM.
	status = _NVM_unlock();
	if (status != NVM_SUCCESS) goto errors;
	// Read data.
	(*data) = *((uint8_t*) (EEPROM_START_ADDRESS + address));
	// Lock NVM.
	status = _NVM_lock();
errors:
	// Disable peripheral.
	RCC -> AHBENR &= ~(0b1 << 8); // MIFEN='0'.
	return status;
}

/*******************************************************************/
NVM_status_t NVM_write_byte(NVM_address_t address, uint8_t data) {
	// Local variables.
	NVM_status_t status = NVM_SUCCESS;
	uint32_t loop_count = 0;
	// Check parameters.
	if (address >= EEPROM_SIZE_BYTES) {
		status = NVM_ERROR_ADDRESS;
		goto errors;
	}
	// Enable peripheral.
	RCC -> AHBENR |= (0b1 << 8); // MIFEN='1'.
	// Unlock NVM.
	status = _NVM_unlock();
	if (status != NVM_SUCCESS) goto errors;
	// Write data.
	(*((uint8_t*) (EEPROM_START_ADDRESS + address))) = data;
	// Wait end of operation.
	while (((FLASH -> SR) & (0b1 << 0)) != 0) {
		// Wait till BSY='1' or timeout.
		loop_count++;
		if (loop_count > NVM_TIMEOUT_COUNT) {
			status = NVM_ERROR_WRITE;
			goto errors;
		}
	}
	// Lock NVM.
	status = _NVM_lock();
errors:
	// Disable peripheral.
	RCC -> AHBENR &= ~(0b1 << 8); // MIFEN='0'.
	return status;
}
