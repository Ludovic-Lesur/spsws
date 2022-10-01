/*
 * i2c_reg.h
 *
 *  Created on: 12 may 2018
 *      Author: Ludo
 */

#ifndef __I2C_REG_H__
#define __I2C_REG_H__

#include "types.h"

/*** I2C registers ***/

typedef struct {
	volatile uint32_t CR1;    		// I2C control register 1.
	volatile uint32_t CR2;    		// I2C control register 2.
	volatile uint32_t OAR1;    		// I2C own address 1 register.
	volatile uint32_t OAR2;    		// I2C own address 2 register.
	volatile uint32_t TIMINGR;  	// I2C timing register.
	volatile uint32_t TIMEOUTR;		// I2C timeout register.
	volatile uint32_t ISR;    		// I2C interrupt and status register.
	volatile uint32_t ICR;    		// I2C interrupt clear register.
	volatile uint32_t PECR;    		// I2C PEC register.
	volatile uint32_t RXDR;    		// I2C receive data register.
	volatile uint32_t TXDR;    		// I2C transmit data register.
} I2C_base_address_t;

/*** I2C base addresses ***/

#define I2C1	((I2C_base_address_t*) ((uint32_t) 0x40005400))
//#define I2C2	((I2C_base_address_t*) ((uint32_t) 0x40005800))
//#define I2C3	((I2C_base_address_t*) ((uint32_t) 0x40007800))

#endif /* __I2C_REG_H__ */
