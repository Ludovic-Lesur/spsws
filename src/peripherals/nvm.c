/*
 * nvm.c
 *
 *  Created on: 19 june 2018
 *      Author: Ludo
 */

#include "nvm.h"

#include "flash_reg.h"
#include "rcc_reg.h"

/*** NVM local functions ***/

/* UNLOCK NVM.
 * @param:	None.
 * @return:	None.
 */
static void NVM_unlock(void) {
	// Check no write/erase operation is running.
	while (((FLASH -> SR) & (0b1 << 0)) != 0); // Wait till BSY='1'.
	// Check the NVM is not allready unlocked.
	if (((FLASH -> PECR) & (0b1 << 0)) != 0) {
		// Perform unlock sequence.
		FLASH -> PEKEYR = 0x89ABCDEF;
		FLASH -> PEKEYR = 0x02030405;
	}
}

/* LOCK NVM.
 * @param:	None.
 * @return:	None.
 */
static void NVM_lock(void) {
	// Check no write/erase operation is running.
	while (((FLASH -> SR) & (0b1 << 0)) != 0); // Wait till BSY='1'.
	// Lock PECR register.
	FLASH -> PECR |= (0b1 << 0); // PELOCK='1'.
}

/*** NVM functions ***/

/* ENABLE NVM INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void NVM_enable(void) {
	// Enable NVM peripheral.
	RCC -> AHBENR |= (0b1 << 8); // MIFEN='1'.
}

/* DISABLE NVM INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void NVM_disable(void) {
	// Disable NVM peripheral.
	RCC -> AHBENR &= ~(0b1 << 8); // MIFEN='1'.
}

/* READ A BYTE STORED IN NVM.
 * @param address_offset:	Address offset starting from NVM start address (expressed in bytes).
 * @param byte_to_read:		Pointer to byte that will contain the value to read.
 * @return:					None.
 */
void NVM_read_byte(unsigned short address_offset, unsigned char* byte_to_read) {
	// Unlock NVM.
	NVM_unlock();
	// Check if address is in EEPROM range.
	if (address_offset < EEPROM_SIZE) {
		(*byte_to_read) = *((unsigned char*) (EEPROM_START_ADDRESS+address_offset)); // Read byte at requested address.
	}
	// Lock NVM.
	NVM_lock();
}

/* WRITE A BYTE TO NVM.
 * @param address_offset:	Address offset starting from NVM start address (expressed in bytes).
 * @param byte_to_store:	Byte to store in NVM.
 * @return:					None.
 */
void NVM_write_byte(unsigned short address_offset, unsigned char byte_to_store) {
	// Unlock NVM.
	NVM_unlock();
	// Check if address is in EEPROM range.
	if (address_offset < EEPROM_SIZE) {
		(*((unsigned char*) (EEPROM_START_ADDRESS+address_offset))) = byte_to_store; // Write byte to requested address.
	}
	// Wait end of operation.
	while (((FLASH -> SR) & (0b1 << 0)) != 0); // Wait till BSY='1'.
	// Lock NVM.
	NVM_lock();
}

/* RESET ALL NVM FIELDS TO DEFAULT VALUE.
 * @param:	None.
 * @return:	None.
 */
void NVM_reset_default(void) {
	// Sigfox parameters.
	NVM_write_byte((NVM_SIGFOX_PN_ADDRESS_OFFSET + 0), 0x00);
	NVM_write_byte((NVM_SIGFOX_PN_ADDRESS_OFFSET + 1), 0x00);
	NVM_write_byte((NVM_SIGFOX_SEQ_ADDRESS_OFFSET + 0), 0x00);
	NVM_write_byte((NVM_SIGFOX_SEQ_ADDRESS_OFFSET + 1), 0x00);
	NVM_write_byte((NVM_SIGFOX_FH_ADDRESS_OFFSET + 0), 0x00);
	NVM_write_byte((NVM_SIGFOX_FH_ADDRESS_OFFSET + 1), 0x00);
	NVM_write_byte(NVM_SIGFOX_RL_ADDRESS_OFFSET, 0x00);
	// Device configuration (mapped on downlink frame).
	NVM_write_byte(NVM_CONFIG_LOCAL_UTC_OFFSET_ADDRESS_OFFSET, 0x01);
	NVM_write_byte(NVM_CONFIG_UPLINK_FRAMES_ADDRESS_OFFSET, 0x00);
	NVM_write_byte(NVM_CONFIG_GPS_TIMEOUT_ADDRESS_OFFSET, 0x78);
	// Period management and status.
	NVM_write_byte(NVM_DAY_COUNT_ADDRESS_OFFSET, 0x01);
	NVM_write_byte(NVM_HOURS_COUNT_ADDRESS_OFFSET, 0x01);
	NVM_write_byte(NVM_MONITORING_STATUS_BYTE_ADDRESS_OFFSET, 0x00);
	// RTC.
	NVM_write_byte((NVM_RTC_PWKUP_YEAR_ADDRESS_OFFSET + 0), 0x00);
	NVM_write_byte((NVM_RTC_PWKUP_YEAR_ADDRESS_OFFSET + 1), 0x00);
	NVM_write_byte(NVM_RTC_PWKUP_MONTH_ADDRESS_OFFSET, 0x00);
	NVM_write_byte(NVM_RTC_PWKUP_DATE_ADDRESS_OFFSET, 0x00);
	NVM_write_byte(NVM_RTC_PWKUP_HOURS_ADDRESS_OFFSET, 0x00);
}
