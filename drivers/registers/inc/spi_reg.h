/*
 * spi_reg.h
 *
 *  Created on: 19 jun. 2018
 *      Author: Ludo
 */

#ifndef __SPI_REG_H__
#define __SPI_REG_H__

#include "types.h"

/*** SPI REG macros ***/

// Peripherals base address.
#define SPI1	((SPI_registers_t*) ((uint32_t) 0x40013000))
#if (defined MCU_CATEGORY_3) || (defined MCU_CATEGORY_5)
#define SPI2	((SPI_registers_t*) ((uint32_t) 0x40003800))
#endif

/*** SPI REG structures ***/

/*!******************************************************************
 * \enum SPI_registers_t
 * \brief SPI registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t CR1;    	// SPI control register 1.
	volatile uint32_t CR2;    	// SPI control register 2.
	volatile uint32_t SR;    	// SPI status register.
	volatile uint32_t DR;    	// SPI data register.
	volatile uint32_t CRCPR;    // SPI CRC polynomial register.
	volatile uint32_t RXCRCR;   // SPI RX CRC register.
	volatile uint32_t TXCRCR;   // SPI TX CRC register.
	volatile uint32_t I2SCFGR;	// SPI I2S configuration register.
	volatile uint32_t I2SPR;    // SPI I2S prescaler register.
} SPI_registers_t;

#endif /* __SPI_REG_H__ */
