/*
 * aes.h
 *
 *  Created on: 19 juin 2018
 *      Author: Ludo
 */

#ifndef __AES_H__
#define __AES_H__

#include "types.h"

/*** AES macros ***/

#define AES_BLOCK_SIZE 	16 // 128-bits is 16 bytes.

/*** AES structures ***/

typedef enum {
	AES_SUCCESS = 0,
	AES_ERROR_TIMEOUT,
	AES_ERROR_BASE_LAST = 0x0100
} AES_status_t;

/*** AES functions ***/

void AES_init(void);
AES_status_t AES_encrypt(uint8_t* data_in, uint8_t* data_out, uint8_t* init_vector, uint8_t* key);

#define AES_status_check(error_base) { if (aes_status != AES_SUCCESS) { status = error_base + aes_status; goto errors; }}
#define AES_error_check() { ERROR_status_check(aes_status, AES_SUCCESS, ERROR_BASE_AES); }
#define AES_error_check_print() { ERROR_status_check_print(aes_status, AES_SUCCESS, ERROR_BASE_AES); }

#endif /* __AES_H__ */
