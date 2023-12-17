/*
 * aes_reg.h
 *
 *  Created on: 19 dec. 2018
 *      Author: Ludo
 */

#ifndef __AES_REG_H__
#define __AES_REG_H__

#include "types.h"

/*** AES REG macros ***/

// Peripheral base address.
#define AES		((AES_registers_t*) ((uint32_t) 0x40026000))

/*** AES REG structures ***/

/*!******************************************************************
 * \enum AES_registers_t
 * \brief AES registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t CR;    			// AES control register.
	volatile uint32_t SR;    			// AES status register.
	volatile uint32_t DINR;    			// AES input data register.
	volatile uint32_t DOUTR;    		// AES output data register.
	union {
		struct {
			volatile uint32_t KEYR0;    // AES key register 0.
			volatile uint32_t KEYR1;    // AES key register 1.
			volatile uint32_t KEYR2;    // AES key register 2.
			volatile uint32_t KEYR3;    // AES key register 3.
		};
		volatile uint32_t KEYR[4];		// AES key registers.
	};
	union {
		struct {
			volatile uint32_t IVR0;    	// AES initialization vector register 0.
			volatile uint32_t IVR1;    	// AES initialization vector register 1.
			volatile uint32_t IVR2;    	// AES initialization vector register 2.
			volatile uint32_t IVR3;    	// AES initialization vector register 3.
		};
		volatile uint32_t IVR[4];		// AES initialization vector registers.
	};
} AES_registers_t;

#endif /* __AES_REG_H__ */
