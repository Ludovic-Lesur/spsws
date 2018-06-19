/*
 * nvm.c
 *
 *  Created on: 19 june 2018
 *      Author: Ludovic
 */

#include "nvm.h"
#include "nvm_reg.h"
#include "rcc_reg.h"

// Sigfox and station parameters are stored in EEPROM according to the following mapping (index 0 = EEPROM_START_ADDRESS):
// ___________________________________________________________________________
// |                                                   |                      |
// |                 Sigfox parameters                 |  Station parameters  |
// |___________________________________________________|______________________|
// |0    3|4    19|20   27|28  29|30   31|32  33|  34  |35                  ..|
// |      |       |       |      |       |      |      |                      |
// |  ID  |  KEY  |  PAC  |  PN  |  SEQ  |  FH  |  RL  |         TBC          |
// |______|_______|_______|______|_______|______|______|______________________|

/*** NVM local macros ***/

// If defined, force Sigfox parameters re-flashing at start-up (values defined in EEPROM_SigfoxParametersReflash() function).
//#define SIGFOX_PARAMETERS_REFLASH
// If defined, force station parameters re-flashing at start-up (values defined in EEPROM_StationParametersReflash() function).
//#define STATION_PARAMETERS_REFLASH

// Sigfox device parameters address offsets.
#define SIGFOX_ID_ADDRESS_OFFSET			0
#define SIGFOX_KEY_ADDRESS_OFFSET			4
#define SIGFOX_INITIAL_PAC_ADDRESS_OFFSET	20
#define SIGFOX_PN_ADDRESS_OFFSET			28
#define SIGFOX_SEQ_NUM_ADDRESS_OFFSET		30
#define SIGFOX_FH_ADDRESS_OFFSET			32
#define SIGFOX_RL_ADDRESS_OFFSET			34

// Station parameters address offsets.
// TBC.

/*** NVM local structures ***/

// Sigfox parameters stored in NVM.
typedef struct {
	unsigned char sigfox_id[SIGFOX_ID_SIZE];
	unsigned char sigfox_key[SIGFOX_KEY_SIZE];
	unsigned char sigfox_initial_pac[SIGFOX_PAC_SIZE];
	unsigned short sigfox_pn;
	unsigned short sigfox_seq_num;
	unsigned short sigfox_fh;
	unsigned char sigfox_rl;
} NVM_SigfoxParameters;

// Station parameters stored in NVM.
typedef struct {
	// TBC.
} NVM_StationParameters;

/*** NVM global variables ***/

static NVM_SigfoxParameters nvm_sigfox_parameters;
//static NVM_StationParameters nvm_station_parameters;

/*** NVM local functions ***/

/* UNLOCK EEPROM.
 * @param:	None.
 * @return:	None.
 */
void EEPROM_Unlock(void) {
	// Check no write/erase operation is running.
	while (((NVMI -> SR) & (0b1 << 0)) != 0); // Wait till BSY='1'.
	// Check the NVM is not allready unlocked.
	if (((NVMI -> PECR) & (0b1 << 0)) != 0) {
		// Perform unlock sequence.
		NVMI -> PEKEYR = 0x89ABCDEF;
		NVMI -> PEKEYR = 0x02030405;
	}
}

/* LOCK EEPROM.
 * @param:	None.
 * @return:	None.
 */
void EEPROM_Lock(void) {
	// Check no write/erase operation is running.
	while (((NVMI -> SR) & (0b1 << 0)) != 0); // Wait till BSY='1'.
	// Lock PECR register.
	NVMI -> PECR |= (0b1 << 0); // PELOCK='1'.
}

/* WRITE A BYTE TO EEPROM.
 * @param address_offset:	Address offset starting from EEPROM start address (expressed in bytes).
 * @param byte_to_store:	Byte to store in EEPROM.
 * @return:					None.
 */
void EEPROM_WriteByte(unsigned short address_offset, unsigned char byte_to_store) {
	// Unlock EEPROM.
	EEPROM_Unlock();
	// Check if address is in EEPROM range.
	if (address_offset < EEPROM_SIZE) {
		(*((unsigned char*) (EEPROM_START_ADDRESS+address_offset))) = byte_to_store; // Write byte to requested address.
	}
	// Wait end of operation.
	while (((NVMI -> SR) & (0b1 << 0)) != 0); // Wait till BSY='1'.
	// Lock EEPROM.
	EEPROM_Lock();
}

/* WRITE A 16-BITS VALUE TO EEPROM.
 * @param address_offset:	Address offset starting from EEPROM start address (expressed in bytes).
 * @param short_to_store:	Short to store in EEPROM.
 * @return:					None.
 */
void EEPROM_WriteShort(unsigned short address_offset, unsigned short short_to_store) {
	// Unlock EEPROM.
	EEPROM_Unlock();
	// Check if address is in EEPROM range.
	if (address_offset < EEPROM_SIZE) {
		(*((unsigned short*) (EEPROM_START_ADDRESS+address_offset))) = short_to_store; // Write short to requested address.
	}
	// Wait end of operation.
	while (((NVMI -> SR) & (0b1 << 0)) != 0); // Wait till BSY='1'.
	// Lock EEPROM.
	EEPROM_Lock();
}

/* READ A BYTE STORED IN EEPROM.
 * @param address_offset:	Address offset starting from EEPROM start address (expressed in bytes).
 * @param byte_to_read:		Pointer to byte that will contain the value to read.
 * @return:					None.
 */
void EEPROM_ReadByte(unsigned short address_offset, unsigned char* byte_to_read) {
	// Unlock EEPROM.
	EEPROM_Unlock();
	// Check if address is in EEPROM range.
	if (address_offset < EEPROM_SIZE) {
		(*byte_to_read) = *((unsigned char*) (EEPROM_START_ADDRESS+address_offset)); // Read byte at requested address.
	}
	// Lock EEPROM.
	EEPROM_Lock();
}

/* READ A 16-BITS VALUE STORED IN EEPROM.
 * @param address_offset:	Address offset starting from EEPROM start address (expressed in bytes).
 * @param short_to_read:	Pointer to short that will contain the value to read.
 * @return:					None.
 */
void EEPROM_ReadShort(unsigned short address_offset, unsigned short* short_to_read) {
	// Unlock EEPROM.
	EEPROM_Unlock();
	// Check if address is in EEPROM range.
	if (address_offset < (EEPROM_SIZE-1)) {
		(*short_to_read) = *((unsigned short*) (EEPROM_START_ADDRESS+address_offset)); // Read short at requested address.
	}
	// Lock EEPROM.
	EEPROM_Lock();
}

#ifdef SIGFOX_PARAMETERS_REFLASH
/* ERASE AND REFLASH SIGFOX PARAMETERS IN EEPROM.
 * @param:	None.
 * @return:	None.
 */
void EEPROM_SigfoxParametersReflash(void) {
	unsigned char byte_idx = 0;

	/* New Sigfox parameters to store */
	unsigned char new_sigfox_id[SIGFOX_ID_SIZE] = {0x00};
	unsigned char new_sigfox_key[SIGFOX_KEY_SIZE] = {0x00};
	unsigned char new_sigfox_initial_pac[SIGFOX_PAC_SIZE] = {0x00};

	/* Erase and reflash all Sigfox parameters */
	// Device ID.
	for (byte_idx=0 ; byte_idx<SIGFOX_ID_SIZE ; byte_idx++) {
		EEPROM_WriteByte(SIGFOX_ID_ADDRESS_OFFSET+byte_idx, new_sigfox_id[byte_idx]);
	}
	// Device key.
	for (byte_idx=0 ; byte_idx<SIGFOX_KEY_SIZE ; byte_idx++) {
		EEPROM_WriteByte(SIGFOX_KEY_ADDRESS_OFFSET+byte_idx, new_sigfox_key[byte_idx]);
	}
	// Initial PAC.
	for (byte_idx=0 ; byte_idx<SIGFOX_PAC_SIZE ; byte_idx++) {
		EEPROM_WriteByte(SIGFOX_INITIAL_PAC_ADDRESS_OFFSET+byte_idx, new_sigfox_initial_pac[byte_idx]);
	}
	// Reset PN.
	EEPROM_WriteShort(SIGFOX_PN_ADDRESS_OFFSET, 0);
	// Reset Sequence number.
	EEPROM_WriteShort(SIGFOX_SEQ_NUM_ADDRESS_OFFSET, 0);
	// Reset FH.
	EEPROM_WriteShort(SIGFOX_FH_ADDRESS_OFFSET, 0);
	// Reset RL.
	EEPROM_WriteByte(SIGFOX_RL_ADDRESS_OFFSET, 0);
}
#endif

/* READ SIGFOX PARAMETERS STORED IN EEPROM.
 * @param:	None.
 * @return:	None.
 */
void EEPROM_SigfoxParametersRead(void) {
	unsigned char byte_idx;

	/* Read Sigfox parameters */
	// Read device ID.
	for (byte_idx=0 ; byte_idx<SIGFOX_ID_SIZE ; byte_idx++) {
		EEPROM_ReadByte(SIGFOX_ID_ADDRESS_OFFSET+byte_idx, &(nvm_sigfox_parameters.sigfox_id[byte_idx]));
	}
	// Read device key.
	for (byte_idx=0 ; byte_idx<SIGFOX_KEY_SIZE ; byte_idx++) {
		EEPROM_ReadByte(SIGFOX_KEY_ADDRESS_OFFSET+byte_idx, &(nvm_sigfox_parameters.sigfox_key[byte_idx]));
	}
	// Read initial PAC.
	for (byte_idx=0 ; byte_idx<SIGFOX_PAC_SIZE ; byte_idx++) {
		EEPROM_ReadByte(SIGFOX_INITIAL_PAC_ADDRESS_OFFSET+byte_idx, &(nvm_sigfox_parameters.sigfox_initial_pac[byte_idx]));
	}
	// Read PN.
	EEPROM_ReadShort(SIGFOX_PN_ADDRESS_OFFSET, &(nvm_sigfox_parameters.sigfox_pn));
	// Read sequence number.
	EEPROM_ReadShort(SIGFOX_SEQ_NUM_ADDRESS_OFFSET, &(nvm_sigfox_parameters.sigfox_seq_num));
	// Read FH.
	EEPROM_ReadShort(SIGFOX_FH_ADDRESS_OFFSET, &(nvm_sigfox_parameters.sigfox_fh));
	// Read RL.
	EEPROM_ReadByte(SIGFOX_RL_ADDRESS_OFFSET, &(nvm_sigfox_parameters.sigfox_rl));
}

#ifdef STATION_PARAMETERS_REFLASH
/* ERASE AND REFLASH STATION PARAMETERS IN EEPROM.
 * @param:	None.
 * @return:	None.
 */
void EEPROM_StationParametersReflash(void) {
	//unsigned char byte_idx = 0;

	/* New station parameters to store */
	// TBC.

	/* Erase and reflash all Sigfox parameters */
	// TBC.
}
#endif

/* READ STATION PARAMETERS STORED IN EEPROM.
 * @param:	None.
 * @return:	None.
 */
void EEPROM_StationParametersRead(void) {
	//unsigned char byte_idx;

	/* Read station parameters */
	// TBC.
}

/*** NVM functions ***/

/* INIT NVM INTERFACE AND READ ALL STORED VARIABLES.
 * @param:	None.
 * @return:	None.
 */
void NVM_Init(void) {

	/* Enable peripheral clock */
	RCC -> AHBENR |= (0b1 << 8); // MIFEN='1'.

	/* Sigfox parameters management */
#ifdef SIGFOX_PARAMETERS_REFLASH
	EEPROM_SigfoxParametersReflash();
#endif
	EEPROM_SigfoxParametersRead();

	/* Station parameters management */
#ifdef STATION_PARAMETERS_REFLASH
	EEPROM_StationParametersReflash();
#endif
	EEPROM_StationParametersRead();
}

/* GET SIGFOX DEVICE ID.
 * @param sfx_id:	Byte array that will contain Sigfox device ID.
 * @return:			None.
 */
void NVM_GetSigfoxId(unsigned char sfx_id[SIGFOX_ID_SIZE]) {
	unsigned char byte_idx = 0;
	for (byte_idx=0 ; byte_idx<SIGFOX_ID_SIZE ; byte_idx++) {
		sfx_id[byte_idx] = nvm_sigfox_parameters.sigfox_id[byte_idx];
	}
}

/* GET SIGFOX DEVICE KEY.
 * @param sfx_key:	Byte array that will contain Sigfox device key.
 * @return:			None.
 */
void NVM_GetSigfoxKey(unsigned char sfx_key[SIGFOX_KEY_SIZE]) {
	unsigned char byte_idx = 0;
	for (byte_idx=0 ; byte_idx<SIGFOX_KEY_SIZE ; byte_idx++) {
		sfx_key[byte_idx] = nvm_sigfox_parameters.sigfox_key[byte_idx];
	}
}

/* GET SIGFOX INITIAL PAC.
 * @param sfx_initial_pac:	Byte array that will contain Sigfox initial PAC.
 * @return:					None.
 */
void NVM_GetSigfoxInitialPac(unsigned char sfx_initial_pac[SIGFOX_ID_SIZE]) {
	unsigned char byte_idx = 0;
	for (byte_idx=0 ; byte_idx<SIGFOX_PAC_SIZE ; byte_idx++) {
		sfx_initial_pac[byte_idx] = nvm_sigfox_parameters.sigfox_initial_pac[byte_idx];
	}
}

/* GET SIGFOX PN
 * @param sfx_pn:	Pointer to short that will contain Sigfox PN.
 * @return:			None.
 */
void NVM_GetSigfoxPn(unsigned short* sfx_pn) {
	(*sfx_pn) = nvm_sigfox_parameters.sigfox_pn;
}

/* GET SIGFOX SEQUENCE NUMBER.
 * @param sfx_pn:	Pointer to short that will contain Sigfox sequence number.
 * @return:			None.
 */
void NVM_GetSigfoxSeqNum(unsigned short* sfx_seq_num) {
	(*sfx_seq_num) = nvm_sigfox_parameters.sigfox_seq_num;
}

/* GET SIGFOX FH.
 * @param sfx_fh:	Pointer to short that will contain Sigfox FH.
 * @return:			None.
 */
void NVM_GetSigfoxFh(unsigned short* sfx_fh) {
	(*sfx_fh) = nvm_sigfox_parameters.sigfox_fh;
}

/* GET SIGFOX RL.
 * @param sfx_rl:	Pointer to byte that will contain Sigfox RL.
 * @return:			None.
 */
void NVM_GetSigfoxRl(unsigned char* sfx_rl) {
	(*sfx_rl) = nvm_sigfox_parameters.sigfox_rl;
}

/* SET AND STORE SIGFOX PN IN NVM.
 * @param new_sfx_pn:	New Sigfox PN value.
 * @return:				None.
 */
void NVM_SetSigfoxPn(unsigned short new_sfx_pn) {
	// Store new value in NVM.
	EEPROM_WriteShort(SIGFOX_PN_ADDRESS_OFFSET, new_sfx_pn);
	// Update structure for next reading operations.
	nvm_sigfox_parameters.sigfox_pn = new_sfx_pn;
}

/* SET AND STORE SIGFOX SEQUENCE NUMBER IN NVM.
 * @param new_sfx_seq_num:	New Sigfox sequence number value.
 * @return:					None.
 */
void NVM_SetSigfoxSeqNum(unsigned short new_sfx_seq_num) {
	// Store new value in NVM.
	EEPROM_WriteShort(SIGFOX_SEQ_NUM_ADDRESS_OFFSET, new_sfx_seq_num);
	// Update structure for next reading operations.
	nvm_sigfox_parameters.sigfox_seq_num = new_sfx_seq_num;
}

/* SET AND STORE SIGFOX FH IN NVM.
 * @param new_sfx_fh:	New Sigfox FH value.
 * @return:				None.
 */
void NVM_SetSigfoxFh(unsigned short new_sfx_fh) {
	// Store new value in NVM.
	EEPROM_WriteShort(SIGFOX_FH_ADDRESS_OFFSET, new_sfx_fh);
	// Update structure for next reading operations.
	nvm_sigfox_parameters.sigfox_fh = new_sfx_fh;
}

/* SET AND STORE SIGFOX RL IN NVM.
 * @param new_sfx_rl:	New Sigfox RL value.
 * @return:				None.
 */
void NVM_SetSigfoxRl(unsigned char new_sfx_rl) {
	// Store new value in NVM.
	EEPROM_WriteShort(SIGFOX_RL_ADDRESS_OFFSET, new_sfx_rl);
	// Update structure for next reading operations.
	nvm_sigfox_parameters.sigfox_rl = new_sfx_rl;
}
