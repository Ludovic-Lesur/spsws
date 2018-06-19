/*
 * aes.h
 *
 *  Created on: 19 juin 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_AES_H_
#define PERIPHERALS_AES_H_

/*** AES macros ***/

#define AES_BLOCK_SIZE 	16

/*** AES functions ***/

void AES_Encode(unsigned char* block, unsigned char* key);

#endif /* PERIPHERALS_AES_H_ */
