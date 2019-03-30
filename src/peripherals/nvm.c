/*
 * nvm.c
 *
 *  Created on: 19 june 2018
 *      Author: Ludo
 */

#include "nvm.h"

#include "nvm_reg.h"
#include "rcc_reg.h"

/*** NVM local functions ***/

/* UNLOCK NVM.
 * @param:	None.
 * @return:	None.
 */
void NVM_Unlock(void) {
	// Check no write/erase operation is running.
	while (((NVMI -> SR) & (0b1 << 0)) != 0); // Wait till BSY='1'.
	// Check the NVM is not allready unlocked.
	if (((NVMI -> PECR) & (0b1 << 0)) != 0) {
		// Perform unlock sequence.
		NVMI -> PEKEYR = 0x89ABCDEF;
		NVMI -> PEKEYR = 0x02030405;
	}
}

/* LOCK NVM.
 * @param:	None.
 * @return:	None.
 */
void NVM_Lock(void) {
	// Check no write/erase operation is running.
	while (((NVMI -> SR) & (0b1 << 0)) != 0); // Wait till BSY='1'.
	// Lock PECR register.
	NVMI -> PECR |= (0b1 << 0); // PELOCK='1'.
}

/*** NVM functions ***/

/* ENABLE NVM INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void NVM_Enable(void) {

	/* Enable NVM peripheral */
	RCC -> AHBENR |= (0b1 << 8); // MIFEN='1'.
}

/* DISABLE NVM INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void NVM_Disable(void) {

	/* Disable NVM peripheral */
	RCC -> AHBENR &= ~(0b1 << 8); // MIFEN='1'.
}

/* READ A BYTE STORED IN NVM.
 * @param address_offset:	Address offset starting from NVM start address (expressed in bytes).
 * @param byte_to_read:		Pointer to byte that will contain the value to read.
 * @return:					None.
 */
void NVM_ReadByte(unsigned short address_offset, unsigned char* byte_to_read) {
	// Unlock NVM.
	NVM_Unlock();
	// Check if address is in EEPROM range.
	if (address_offset < EEPROM_SIZE) {
		(*byte_to_read) = *((unsigned char*) (EEPROM_START_ADDRESS+address_offset)); // Read byte at requested address.
	}
	// Lock NVM.
	NVM_Lock();
}

/* WRITE A BYTE TO NVM.
 * @param address_offset:	Address offset starting from NVM start address (expressed in bytes).
 * @param byte_to_store:	Byte to store in NVM.
 * @return:					None.
 */
void NVM_WriteByte(unsigned short address_offset, unsigned char byte_to_store) {
	// Unlock NVM.
	NVM_Unlock();
	// Check if address is in EEPROM range.
	if (address_offset < EEPROM_SIZE) {
		(*((unsigned char*) (EEPROM_START_ADDRESS+address_offset))) = byte_to_store; // Write byte to requested address.
	}
	// Wait end of operation.
	while (((NVMI -> SR) & (0b1 << 0)) != 0); // Wait till BSY='1'.
	// Lock NVM.
	NVM_Lock();
}

/* RESET ALL NVM FIELDS TO DEFAULT VALUE.
 * @param:	None.
 * @return:	None.
 */
void NVM_ResetDefault(void) {
	// Sigfox device ID.
	NVM_WriteByte((NVM_SIGFOX_ID_ADDRESS_OFFSET + 0), 0xFE);
	NVM_WriteByte((NVM_SIGFOX_ID_ADDRESS_OFFSET + 1), 0xDC);
	NVM_WriteByte((NVM_SIGFOX_ID_ADDRESS_OFFSET + 2), 0xBA);
	NVM_WriteByte((NVM_SIGFOX_ID_ADDRESS_OFFSET + 3), 0x98);
	// Sigfox device private key.
	NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + 0), 0x00);
	NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + 1), 0x11);
	NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + 2), 0x22);
	NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + 3), 0x33);
	NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + 4), 0x44);
	NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + 5), 0x55);
	NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + 6), 0x66);
	NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + 7), 0x77);
	NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + 8), 0x88);
	NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + 9), 0x99);
	NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + 10), 0xAA);
	NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + 11), 0xBB);
	NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + 12), 0xCC);
	NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + 13), 0xDD);
	NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + 14), 0xEE);
	NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + 15), 0xFF);
	// Sigfox PN.
	NVM_WriteByte((NVM_SIGFOX_PN_ADDRESS_OFFSET + 0), 0x00);
	NVM_WriteByte((NVM_SIGFOX_PN_ADDRESS_OFFSET + 1), 0x00);
	// Sigfox SEQ.
	NVM_WriteByte((NVM_SIGFOX_SEQ_ADDRESS_OFFSET + 0), 0x00);
	NVM_WriteByte((NVM_SIGFOX_SEQ_ADDRESS_OFFSET + 1), 0x00);
	// Sigfox FH.
	NVM_WriteByte((NVM_SIGFOX_FH_ADDRESS_OFFSET + 0), 0x00);
	NVM_WriteByte((NVM_SIGFOX_FH_ADDRESS_OFFSET + 1), 0x00);
	// Sigfox RL.
	NVM_WriteByte(NVM_SIGFOX_RL_ADDRESS_OFFSET, 0x00);
	// Device configuration (mapped on downlink frame).
	NVM_WriteByte(NVM_CONFIG_LOCAL_UTC_OFFSET_ADDRESS_OFFSET, 0x01);
	NVM_WriteByte(NVM_CONFIG_UPLINK_FRAMES_ADDRESS_OFFSET, 0x00);
	NVM_WriteByte(NVM_CONFIG_GPS_TIMEOUT_ADDRESS_OFFSET, 0x78);
	// Period management and status.
	NVM_WriteByte(NVM_DAY_COUNT_ADDRESS_OFFSET, 0x01);
	NVM_WriteByte(NVM_HOURS_COUNT_ADDRESS_OFFSET, 0x01);
	NVM_WriteByte(NVM_MONITORING_STATUS_BYTE_ADDRESS_OFFSET, 0x00);
	// RTC.
	NVM_WriteByte((NVM_RTC_PWKUP_YEAR_ADDRESS_OFFSET + 0), 0x00);
	NVM_WriteByte((NVM_RTC_PWKUP_YEAR_ADDRESS_OFFSET + 1), 0x00);
	NVM_WriteByte(NVM_RTC_PWKUP_MONTH_ADDRESS_OFFSET, 0x00);
	NVM_WriteByte(NVM_RTC_PWKUP_DATE_ADDRESS_OFFSET, 0x00);
	NVM_WriteByte(NVM_RTC_PWKUP_HOURS_ADDRESS_OFFSET, 0x00);
#ifdef IM_HWT
	// HWT.
	NVM_WriteByte((NVM_HWT_PGT_YEAR_ADDRESS_OFFSET + 0), 0x00);
	NVM_WriteByte((NVM_HWT_PGT_YEAR_ADDRESS_OFFSET + 1), 0x00);
	NVM_WriteByte(NVM_HWT_PGT_MONTH_ADDRESS_OFFSET, 0x00);
	NVM_WriteByte(NVM_HWT_PGT_DATE_ADDRESS_OFFSET, 0x00);
	NVM_WriteByte(NVM_HWT_PGT_HOURS_ADDRESS_OFFSET, 0x00);
	NVM_WriteByte(NVM_HWT_PGT_MINUTES_ADDRESS_OFFSET, 0x00);
	NVM_WriteByte(NVM_HWT_PGT_SECONDS_ADDRESS_OFFSET, 0x00);
	NVM_WriteByte((NVM_HWT_PGT_MCU_TIME_ADDRESS_OFFSET + 0), 0x00);
	NVM_WriteByte((NVM_HWT_PGT_MCU_TIME_ADDRESS_OFFSET + 1), 0x00);
	NVM_WriteByte((NVM_HWT_MAX5495_VALUE_ADDRESS_OFFSET + 0), 0x00);
	NVM_WriteByte((NVM_HWT_MAX5495_VALUE_ADDRESS_OFFSET + 1), 0x00);
#endif
}
