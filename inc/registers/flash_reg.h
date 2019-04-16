/*
 * flash_reg.h
 *
 *  Created on: 31 march 2019
 *      Author: Ludovic
 */

#ifndef FLASH_REG_H
#define FLASH_REG_H

/*** FLASH registers ***/

typedef struct {
	volatile unsigned int ACR;    	// FLASH access control register.
	volatile unsigned int PECR;    	// FLASH program an erase control register.
	volatile unsigned int PDKEYR;   // FLASH power-down key register.
	volatile unsigned int PKEYR;    // FLASH PECR unlock key register.
	volatile unsigned int PRGKEYR;  // FLASH program and erase key register.
	volatile unsigned int OPTKEYR;  // FLASH option bytes unlock key register.
	volatile unsigned int SR;    	// FLASH status register.
	volatile unsigned int OPTR;    	// FLASH option bytes register.
	volatile unsigned int WRPROT1;  // FLASH write protection register 1.
	unsigned int RESERVED[23];		// 0x24 to 0x78.
	volatile unsigned int WRPROT2;  // FLASH write protection register 2.
} FLASH_BaseAddress;

/*** FLASH base address ***/

#define FLASH	((FLASH_BaseAddress*) ((unsigned int) 0x40022000))

#endif /* FLASH_REG_H */
