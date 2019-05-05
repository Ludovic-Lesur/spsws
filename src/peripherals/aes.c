/*
 * aes.c
 *
 *  Created on: 19 juin 2018
 *      Author: Ludo
 */

#include "aes.h"

#include "aes_reg.h"
#include "rcc_reg.h"

/*** AES functions ***/

/* INIT AES HARDWARE PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void AES_Init(void) {

	/* Enable peripheral clock */
	RCC -> AHBENR |= (0b1 << 24); // CRYPTOEN='1'.

	/* Configure peripheral */
	AES -> CR &= 0xFFFFE000; // Disable peripheral.
	AES -> CR |= (0b01 << 5); // CBC algorithme (CHMOD='01').
	AES -> CR &= ~(0b11 << 1); // No swapping (DATATYPE='00').
}

/* DISABLE AES PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void AES_Disable(void) {

	/* Disable AES peripheral */
	RCC -> AHBENR &= ~(0b1 << 24); // CRYPTOEN='0'.
}

/* COMPUTE AES-128 CBC ALGORITHME WITH HARDWARE ACCELERATOR.
 * @param data_in:		Input data (16-bits value).
 * @param init_vector:	Initialisation vector (128-bits value).
 * @param key			AES key (128-bits value).
 */
void AES_EncodeCbc(unsigned char data_in[AES_BLOCK_SIZE], unsigned char data_out[AES_BLOCK_SIZE], unsigned char init_vector[AES_BLOCK_SIZE], unsigned char key[AES_BLOCK_SIZE]) {

	/* Configure operation */
	AES -> CR &= ~(0b11 << 3); // MODE='00'.

	/* Fill key */
	AES -> KEYR3 = 0;
	AES -> KEYR3 |= (key[0] << 24) | (key[1] << 16) | (key[2] << 8) | (key[3] << 0);
	AES -> KEYR2 = 0;
	AES -> KEYR2 |= (key[4] << 24) | (key[5] << 16) | (key[6] << 8) | (key[7] << 0);
	AES -> KEYR1 = 0;
	AES -> KEYR1 |= (key[8] << 24) | (key[9] << 16) | (key[10] << 8) | (key[11] << 0);
	AES -> KEYR0 = 0;
	AES -> KEYR0 |= (key[12] << 24) | (key[13] << 16) | (key[14] << 8) | (key[15] << 0);

	/* Fill initialization vector */
	AES -> IVR3 = 0;
	AES -> IVR3 |= (init_vector[0] << 24) | (init_vector[1] << 16) | (init_vector[2] << 8) | (init_vector[3] << 0);
	AES -> IVR2 = 0;
	AES -> IVR2 |= (init_vector[4] << 24) | (init_vector[5] << 16) | (init_vector[6] << 8) | (init_vector[7] << 0);
	AES -> IVR1 = 0;
	AES -> IVR1 |= (init_vector[8] << 24) | (init_vector[9] << 16) | (init_vector[10] << 8) | (init_vector[11] << 0);
	AES -> IVR0 = 0;
	AES -> IVR0 |= (init_vector[12] << 24) | (init_vector[13] << 16) | (init_vector[14] << 8) | (init_vector[15] << 0);

	/* Enable peripheral */
	AES -> CR |= (0b1 << 0); // EN='1'.

	/* Fill input data (provided most significant 32-bits word first) */
	unsigned char register_idx = 0;
	for (register_idx=0 ; register_idx<4 ; register_idx++) {
		// Build 32-bits word.
		unsigned int data_in_32bits = 0;
		data_in_32bits |= data_in[(register_idx*4)+0] << 24;
		data_in_32bits |= data_in[(register_idx*4)+1] << 16;
		data_in_32bits |= data_in[(register_idx*4)+2] << 8;
		data_in_32bits |= data_in[(register_idx*4)+3] << 0;
		// Fill input data register (most significant 32-bits word first).
		AES -> DINR = data_in_32bits;
	}

	/* Wait for algorithme to complete */
	while (((AES -> SR) & (0b1 << 0)) == 0); // Wait for CCF='1'.

	/* Get result (returned most signifiant 32-bits word first) */
	for (register_idx=0 ; register_idx<4 ; register_idx++) {
		// Read output data register (most signifiant 32-bits word first).
		unsigned int data_out_32bits = AES -> DOUTR;
		// Split 32-bits word into 4 bytes.
		data_out[(register_idx*4)+0] = (data_out_32bits & 0xFF000000) >> 24;
		data_out[(register_idx*4)+1] = (data_out_32bits & 0x00FF0000) >> 16;
		data_out[(register_idx*4)+2] = (data_out_32bits & 0x0000FF00) >> 8;
		data_out[(register_idx*4)+3] = (data_out_32bits & 0x000000FF) >> 0;
	}

	/* Reset peripheral */
	AES -> CR |= (0b1 << 7); // Clear CCF flag for next AES operation.
	AES -> CR &= ~(0b1 << 0); // EN='0'.
}
