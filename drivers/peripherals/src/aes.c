/*
 * aes.c
 *
 *  Created on: 19 jun. 2018
 *      Author: Ludo
 */

#include "aes.h"

#include "aes_reg.h"
#include "rcc_reg.h"
#include "types.h"

/*** AES local macros ***/

#define AES_TIMEOUT_COUNT	1000000

/*** AES functions ***/

/*******************************************************************/
void AES_init(void) {
	// Enable peripheral clock.
	RCC -> AHBENR |= (0b1 << 24); // CRYPTOEN='1'.
	// Configure peripheral.
	AES -> CR |= (0b01 << 5); // CBC algorithm (CHMOD='01').
}

/*******************************************************************/
void AES_de_init(void) {
	// Disable peripheral clock.
	RCC -> AHBENR &= ~(0b1 << 24); // CRYPTOEN='0'.
}

/*******************************************************************/
AES_status_t AES_encrypt(uint8_t* data_in, uint8_t* data_out, uint8_t* key) {
	// Local variables.
	AES_status_t status = AES_SUCCESS;
	uint8_t idx = 0;
	uint32_t data_32bits = 0;
	uint32_t loop_count = 0;
	// Check parameters.
	if ((data_in == NULL) || (data_out == NULL) || (key == NULL)) {
		status = AES_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Fill key and reset initialization vector.
	for (idx=0 ; idx<4 ; idx++) {
		// Build 32-bits word.
		data_32bits = 0;
		data_32bits |= key[(idx << 2) + 0] << 24;
		data_32bits |= key[(idx << 2) + 1] << 16;
		data_32bits |= key[(idx << 2) + 2] << 8;
		data_32bits |= key[(idx << 2) + 3] << 0;
		// Fill registers.
		AES -> KEYR[3 - idx] = data_32bits;
		AES -> IVR[idx] = 0;
	}
	// Enable peripheral.
	AES -> CR |= (0b1 << 0); // EN='1'.
	// Fill input data (provided most significant 32-bits word first).
	for (idx=0 ; idx<4 ; idx++) {
		// Build 32-bits word.
		data_32bits = 0;
		data_32bits |= data_in[(idx << 2) + 0] << 24;
		data_32bits |= data_in[(idx << 2) + 1] << 16;
		data_32bits |= data_in[(idx << 2) + 2] << 8;
		data_32bits |= data_in[(idx << 2) + 3] << 0;
		// Fill input data register.
		AES -> DINR = data_32bits;
	}
	// Wait for algorithm to complete.
	while (((AES -> SR) & (0b1 << 0)) == 0) {
		// Wait for CCF='1' or timeout.
		loop_count++;
		if (loop_count > AES_TIMEOUT_COUNT) {
			status = AES_ERROR_TIMEOUT;
			goto errors;
		}
	}
	// Get result (returned most significant 32-bits word first).
	for (idx=0 ; idx<4 ; idx++) {
		// Read output data register (most significant 32-bits word first).
		data_32bits = (AES -> DOUTR);
		// Split 32-bits word into 4 bytes.
		data_out[(idx << 2) + 0] = (data_32bits >> 24) & 0xFF;
		data_out[(idx << 2) + 1] = (data_32bits >> 16) & 0xFF;
		data_out[(idx << 2) + 2] = (data_32bits >> 8) & 0xFF;
		data_out[(idx << 2) + 3] = (data_32bits >> 0) & 0xFF;
	}
	// Reset peripheral.
	AES -> CR |= (0b1 << 7); // Clear CCF flag for next AES operation.
	AES -> CR &= ~(0b1 << 0); // EN='0'.
errors:
	return status;
}
