/*
 * aes.c
 *
 *  Created on: 19 juin 2018
 *      Author: Ludo
 */

#include "aes.h"

#include "aes_reg.h"
#include "rcc_reg.h"
#include "types.h"

/*** AES local macros ***/

#define AES_TIMEOUT_COUNT	1000000

/*** AES functions ***/

/* INIT AES HARDWARE PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void AES_init(void) {
	// Enable peripheral clock.
	RCC -> AHBENR |= (0b1 << 24); // CRYPTOEN='1'.
	// Configure peripheral.
	AES -> CR |= (0b01 << 5); // CBC algorithm (CHMOD='01').
}

/* COMPUTE AES-128 CBC ALGORITHME WITH HARDWARE ACCELERATOR.
 * @param data_in:		Input data (16-bytes array).
 * @param data_out:		Output data (16-bytes array).
 * @param init_vector:	Initialisation vector (16-bytes array).
 * @param key			AES key (16-bytes array).
 * @return status:		Function execution status.
 */
AES_status_t AES_encrypt(uint8_t* data_in, uint8_t* data_out, uint8_t* init_vector, uint8_t* key) {
	// Local variables.
	AES_status_t status = AES_SUCCESS;
	uint8_t idx = 0;
	uint32_t data_32bits = 0;
	uint32_t loop_count = 0;
	// Fill key.
	AES -> KEYR3 = (key[0] << 24) | (key[1] << 16) | (key[2] << 8) | (key[3] << 0);
	AES -> KEYR2 = (key[4] << 24) | (key[5] << 16) | (key[6] << 8) | (key[7] << 0);
	AES -> KEYR1 = (key[8] << 24) | (key[9] << 16) | (key[10] << 8) | (key[11] << 0);
	AES -> KEYR0 = (key[12] << 24) | (key[13] << 16) | (key[14] << 8) | (key[15] << 0);
	// Fill initialization vector.
	AES -> IVR3 = (init_vector[0] << 24) | (init_vector[1] << 16) | (init_vector[2] << 8) | (init_vector[3] << 0);
	AES -> IVR2 = (init_vector[4] << 24) | (init_vector[5] << 16) | (init_vector[6] << 8) | (init_vector[7] << 0);
	AES -> IVR1 = (init_vector[8] << 24) | (init_vector[9] << 16) | (init_vector[10] << 8) | (init_vector[11] << 0);
	AES -> IVR0 = (init_vector[12] << 24) | (init_vector[13] << 16) | (init_vector[14] << 8) | (init_vector[15] << 0);
	// Enable peripheral.
	AES -> CR |= (0b1 << 0); // EN='1'.
	// Fill input data (provided most significant 32-bits word first).
	for (idx=0 ; idx<4 ; idx++) {
		// Build 32-bits word.
		data_32bits = 0;
		data_32bits |= data_in[(idx * 4) + 0] << 24;
		data_32bits |= data_in[(idx * 4) + 1] << 16;
		data_32bits |= data_in[(idx * 4) + 2] << 8;
		data_32bits |= data_in[(idx * 4) + 3] << 0;
		// Fill input data register (most significant 32-bits word first).
		AES -> DINR = data_32bits;
	}
	// Wait for algorithme to complete.
	while (((AES -> SR) & (0b1 << 0)) == 0) {
		// Wait for CCF='1' or timeout.
		loop_count++;
		if (loop_count > AES_TIMEOUT_COUNT) {
			status = AES_ERROR_TIMEOUT;
			goto errors;
		}
	}
	// Get result (returned most signifiant 32-bits word first).
	for (idx=0 ; idx<4 ; idx++) {
		// Read output data register (most signifiant 32-bits word first).
		data_32bits = AES -> DOUTR;
		// Split 32-bits word into 4 bytes.
		data_out[(idx * 4) + 0] = (data_32bits & 0xFF000000) >> 24;
		data_out[(idx * 4) + 1] = (data_32bits & 0x00FF0000) >> 16;
		data_out[(idx * 4) + 2] = (data_32bits & 0x0000FF00) >> 8;
		data_out[(idx * 4) + 3] = (data_32bits & 0x000000FF) >> 0;
	}
	// Reset peripheral.
	AES -> CR |= (0b1 << 7); // Clear CCF flag for next AES operation.
	AES -> CR &= ~(0b1 << 0); // EN='0'.
errors:
	return status;
}
