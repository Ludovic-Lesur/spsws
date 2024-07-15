/*
 * nvm_reg.h
 *
 *  Created on: 19 jun. 2018
 *      Author: Ludo
 */

#ifndef __FLASH_REG_H__
#define __FLASH_REG_H__

#include "types.h"

/*** FLASH REG macros ***/

// Peripheral base address.
#define FLASH					((FLASH_registers_t*) ((uint32_t) 0x40022000))
// EEPROM base address and size.
#define EEPROM_START_ADDRESS	(uint32_t) 0x08080000
#ifdef MCU_CATEGORY_1
#define EEPROM_SIZE_BYTES		512
#endif
#ifdef MCU_CATEGORY_2
#define EEPROM_SIZE_BYTES		1024
#endif
#ifdef MCU_CATEGORY_3
#define EEPROM_SIZE_BYTES		2048
#endif
#ifdef MCU_CATEGORY_5
#define EEPROM_SIZE_BYTES		6144
#endif

/*** FLASH REG structures ***/

/*!******************************************************************
 * \enum FLASH_registers_t
 * \brief FLASH registers map.
 *******************************************************************/
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
} FLASH_registers_t;

#endif /* __FLASH_REG_H__ */
