/*
 * nvm.c
 *
 *  Created on: 19 june 2018
 *      Author: Ludo
 */

#include "nvm.h"

#include "flash_reg.h"
#include "rcc_reg.h"
#include "types.h"

/*** NVM local macros ***/

#define NVM_TIMEOUT_COUNT	1000000

/*** NVM local functions ***/

/* UNLOCK NVM.
 * @param:			None.
 * @return status:	Function execution status.
 */
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
	// Check the NVM is not allready unlocked.
	if (((FLASH -> PECR) & (0b1 << 0)) != 0) {
		// Perform unlock sequence.
		FLASH -> PEKEYR = 0x89ABCDEF;
		FLASH -> PEKEYR = 0x02030405;
	}
errors:
	return status;
}

/* LOCK NVM.
 * @param:			None.
 * @return status:	Function execution status.
 */
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

/* ENABLE NVM INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void NVM_init(void) {
	// Enable NVM peripheral.
	RCC -> AHBENR |= (0b1 << 8); // MIFEN='1'.
}

/* READ A BYTE STORED IN NVM.
 * @param address_offset:	Address offset starting from NVM start address (expressed in bytes).
 * @param data:				Pointer to 8-bits value that will contain the value to read.
 * @return status:			Function execution status.
 */
NVM_status_t NVM_read_byte(NVM_address_t address_offset, uint8_t* data) {
	// Local variables.
	NVM_status_t status = NVM_SUCCESS;
	// Check address.
	if (address_offset >= EEPROM_SIZE) {
		status = NVM_ERROR_ADDRESS;
		goto errors;
	}
	// Unlock NVM.
	status = _NVM_unlock();
	if (status != NVM_SUCCESS) goto errors;
	// Read data.
	(*data) = *((uint8_t*) (EEPROM_START_ADDRESS + address_offset));
	// Lock NVM.
	status = _NVM_lock();
errors:
	return status;
}

/* WRITE A BYTE TO NVM.
 * @param address_offset:	Address offset starting from NVM start address (expressed in bytes).
 * @param data:				Byte to store in NVM.
 * @return status:			Function execution status.
 */
NVM_status_t NVM_write_byte(NVM_address_t address_offset, uint8_t data) {
	// Local variables.
	NVM_status_t status = NVM_SUCCESS;
	uint32_t loop_count = 0;
	// Check address.
	if (address_offset >= EEPROM_SIZE) {
		status = NVM_ERROR_ADDRESS;
		goto errors;
	}
	// Unlock NVM.
	status = _NVM_unlock();
	if (status != NVM_SUCCESS) goto errors;
	// Write data.
	(*((uint8_t*) (EEPROM_START_ADDRESS + address_offset))) = data;
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
	return status;
}

/* RESET ALL NVM FIELDS TO DEFAULT VALUE.
 * @param:			None.
 * @return status:	Function execution status.
 */
NVM_status_t NVM_reset_default(void) {
	// Local variables.
	NVM_status_t status = NVM_SUCCESS;
	// Sigfox parameters.
	status = NVM_write_byte((NVM_ADDRESS_SIGFOX_PN + 0), 0x00);
	if (status != NVM_SUCCESS) goto errors;
	status = NVM_write_byte((NVM_ADDRESS_SIGFOX_PN + 1), 0x00);
	if (status != NVM_SUCCESS) goto errors;
	status = NVM_write_byte((NVM_ADDRESS_SIGFOX_MESSAGE_COUNTER + 0), 0x00);
	if (status != NVM_SUCCESS) goto errors;
	status = NVM_write_byte((NVM_ADDRESS_SIGFOX_MESSAGE_COUNTER + 1), 0x00);
	if (status != NVM_SUCCESS) goto errors;
	status = NVM_write_byte((NVM_ADDRESS_SIGFOX_FH + 0), 0x00);
	if (status != NVM_SUCCESS) goto errors;
	status = NVM_write_byte((NVM_ADDRESS_SIGFOX_FH + 1), 0x00);
	if (status != NVM_SUCCESS) goto errors;
	status = NVM_write_byte(NVM_ADDRESS_SIGFOX_RL, 0x00);
	if (status != NVM_SUCCESS) goto errors;
	// Period management and status.
	status = NVM_write_byte(NVM_ADDRESS_GEOLOC_DATA_DAY_COUNT, 0x01);
	if (status != NVM_SUCCESS) goto errors;
	status = NVM_write_byte(NVM_ADDRESS_WEATHER_DATA_HOUR_COUNT, 0x01);
	if (status != NVM_SUCCESS) goto errors;
	status = NVM_write_byte(NVM_ADDRESS_STATUS, 0x00);
	if (status != NVM_SUCCESS) goto errors;
	// RTC.
	status = NVM_write_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 0), 0x00);
	if (status != NVM_SUCCESS) goto errors;
	status = NVM_write_byte((NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR + 1), 0x00);
	if (status != NVM_SUCCESS) goto errors;
	status = NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_MONTH, 0x00);
	if (status != NVM_SUCCESS) goto errors;
	status = NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_DATE, 0x00);
	if (status != NVM_SUCCESS) goto errors;
	status = NVM_write_byte(NVM_ADDRESS_PREVIOUS_WAKE_UP_HOUR, 0x00);
	if (status != NVM_SUCCESS) goto errors;
errors:
	return status;
}
