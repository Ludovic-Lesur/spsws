/*
 * nvm.c
 *
 *  Created on: 19 june 2018
 *      Author: Ludovic
 */

#include "nvm.h"

#include "nvm_reg.h"
#include "rcc_reg.h"

/*** NVM local macros ***/

// If defined, force Sigfox parameters re-flashing at start-up (values defined in NVM_SigfoxParametersReflash() function).
//#define SIGFOX_PARAMETERS_REFLASH
// If defined, force station parameters re-flashing at start-up (values defined in NVM_StationParametersReflash() function).
//#define STATION_PARAMETERS_REFLASH

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

#ifdef SIGFOX_PARAMETERS_REFLASH
/* ERASE AND REFLASH SIGFOX PARAMETERS IN NVM.
 * @param:	None.
 * @return:	None.
 */
void NVM_SigfoxParametersReflash(void) {
	unsigned char byte_idx = 0;

	/* New Sigfox parameters to store */
	unsigned char new_sigfox_id[SIGFOX_ID_SIZE] = {0x00};
	unsigned char new_sigfox_key[SIGFOX_KEY_SIZE] = {0x00};
	unsigned char new_sigfox_initial_pac[SIGFOX_PAC_SIZE] = {0x00};

	/* Erase and reflash all Sigfox parameters */
	// Device ID.
	for (byte_idx=0 ; byte_idx<SIGFOX_ID_SIZE ; byte_idx++) {
		NVM_WriteByte(SIGFOX_ID_ADDRESS_OFFSET+byte_idx, new_sigfox_id[byte_idx]);
	}
	// Device key.
	for (byte_idx=0 ; byte_idx<SIGFOX_KEY_SIZE ; byte_idx++) {
		NVM_WriteByte(SIGFOX_KEY_ADDRESS_OFFSET+byte_idx, new_sigfox_key[byte_idx]);
	}
	// Initial PAC.
	for (byte_idx=0 ; byte_idx<SIGFOX_PAC_SIZE ; byte_idx++) {
		NVM_WriteByte(SIGFOX_INITIAL_PAC_ADDRESS_OFFSET+byte_idx, new_sigfox_initial_pac[byte_idx]);
	}
	// Reset PN.
	NVM_WriteShort(SIGFOX_PN_ADDRESS_OFFSET, 0);
	// Reset Sequence number.
	NVM_WriteShort(SIGFOX_SEQ_NUM_ADDRESS_OFFSET, 0);
	// Reset FH.
	NVM_WriteShort(SIGFOX_FH_ADDRESS_OFFSET, 0);
	// Reset RL.
	NVM_WriteByte(SIGFOX_RL_ADDRESS_OFFSET, 0);
}
#endif

#ifdef STATION_PARAMETERS_REFLASH
/* ERASE AND REFLASH STATION PARAMETERS IN NVM.
 * @param:	None.
 * @return:	None.
 */
void NVM_StationParametersReflash(void) {
	//unsigned char byte_idx = 0;

	/* New station parameters to store */
	// TBC.

	/* Erase and reflash all Sigfox parameters */
	// TBC.
}
#endif

/*** NVM functions ***/

/* INIT NVM INTERFACE AND REFLASH PARAMETERS IF REQUIRED.
 * @param:	None.
 * @return:	None.
 */
void NVM_Init(void) {

	/* Enable peripheral clock */
	RCC -> AHBENR |= (0b1 << 8); // MIFEN='1'.

	/* Sigfox parameters management */
#ifdef SIGFOX_PARAMETERS_REFLASH
	NVM_SigfoxParametersReflash();
#endif

	/* Station parameters management */
#ifdef STATION_PARAMETERS_REFLASH
	NVM_StationParametersReflash();
#endif
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
