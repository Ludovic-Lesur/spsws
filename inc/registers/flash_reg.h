/*
 * nvm_reg.h
 *
 *  Created on: 19 june 2018
 *      Author: Ludo
 */

#ifndef __FLASH_REG_H__
#define __FLASH_REG_H__

#include "types.h"

/*** FLASH registers ***/

typedef struct {
	volatile uint32_t ACR;			// NVM interface access control register.
	volatile uint32_t PECR;			// NVM interface program and erase control register.
	volatile uint32_t PDKEYR;		// NVM interface power down key register.
	volatile uint32_t PEKEYR;		// NVM interface PECR unlock key register.
	volatile uint32_t PRGKEYR;		// NVM interface program and erase key register.
	volatile uint32_t OPTKEYR;		// NVM interface option bytes unlock key register.
	volatile uint32_t SR;			// NVM interface status register.
	volatile uint32_t OPTR;			// NVM interface option bytes register.
	volatile uint32_t WRPROT1;		// NVM interface write protection register 1.
	volatile uint32_t RESERVED[23];	// Reserved 0x24.
	volatile uint32_t WRPROT2;		// NVM interface write protection register 2.
} FLASH_base_address_t;

/*** FLASH registers base address ***/

#define FLASH	((FLASH_base_address_t*) ((uint32_t) 0x40022000))

/*** EEPROM address range ***/

#define EEPROM_START_ADDRESS	(uint32_t) 0x08080000
#ifdef HW1_0
// EEPROM size is 1kB for STM32L041xxxx (category 2 device).
#define EEPROM_SIZE				1024 // In bytes.
#endif
#ifdef HW2_0
// EEPROM size is 6kB for STM32L081xxxx (category 5 device).
#define EEPROM_SIZE				6144 // In bytes.
#endif

#endif /* __FLASH_REG_H__ */
