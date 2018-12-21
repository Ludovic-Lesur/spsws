/*
 * aes.h
 *
 *  Created on: 19 juin 2018
 *      Author: Ludovic
 */

#ifndef AES_H
#define AES_H

/*** AES macros ***/

#define AES_BLOCK_SIZE 	16 // 128-bits is 16 bytes.

/*** AES functions ***/

void AES_Init(void);
void AES_EncodeCbc(unsigned char data_in[AES_BLOCK_SIZE], unsigned char data_out[AES_BLOCK_SIZE], unsigned char init_vector[AES_BLOCK_SIZE], unsigned char key[AES_BLOCK_SIZE]);
void AES_Encode(unsigned char* block, unsigned char* key);

#endif /* PERIPHERALS_AES_H_ */
