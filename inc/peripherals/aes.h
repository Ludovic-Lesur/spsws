/*
 * aes.h
 *
 *  Created on: 19 juin 2018
 *      Author: Ludo
 */

#ifndef AES_H
#define AES_H

/*** AES macros ***/

#define AES_BLOCK_SIZE 	16 // 128-bits is 16 bytes.

/*** AES structures ***/

typedef enum {
	AES_SUCCESS = 0,
	AES_ERROR_TIMEOUT,
	AES_ERROR_LAST
} AES_status_t;

/*** AES functions ***/

void AES_init(void);
void AES_disable(void);
AES_status_t AES_encode_cbc(unsigned char data_in[AES_BLOCK_SIZE], unsigned char data_out[AES_BLOCK_SIZE], unsigned char init_vector[AES_BLOCK_SIZE], unsigned char key[AES_BLOCK_SIZE]);

#define AES_status_check(error_base) { if (aes_status != AES_SUCCESS) { status = error_base + aes_status; goto errors; }}

#endif /* AES_H_ */
