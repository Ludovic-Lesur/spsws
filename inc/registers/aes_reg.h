/*
 * aes_reg.h
 *
 *  Created on: 19 dec. 2018
 *      Author: Ludo
 */

#ifndef __AES_REG_H__
#define __AES_REG_H__

/*** AES registers ***/

typedef struct {
	volatile unsigned int CR;    	// AES control register.
	volatile unsigned int SR;    	// AES status register.
	volatile unsigned int DINR;    	// AES input data register.
	volatile unsigned int DOUTR;    // AES output data register.
	volatile unsigned int KEYR0;    // AES key register 0.
	volatile unsigned int KEYR1;    // AES key register 1.
	volatile unsigned int KEYR2;    // AES key register 2.
	volatile unsigned int KEYR3;    // AES key register 3.
	volatile unsigned int IVR0;    	// AES initialization vector register 0.
	volatile unsigned int IVR1;    	// AES initialization vector register 1.
	volatile unsigned int IVR2;    	// AES initialization vector register 2.
	volatile unsigned int IVR3;    	// AES initialization vector register 3.
} AES_base_address_t;

/*** AES base address ***/

#define AES		((AES_base_address_t*) ((unsigned int) 0x40026000))

#endif /* __AES_REG_H__ */
