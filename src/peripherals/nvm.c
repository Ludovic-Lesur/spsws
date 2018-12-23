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
