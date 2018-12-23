/*
 * i2c_reg.h
 *
 *  Created on: 12 may 2018
 *      Author: Ludo
 */

#ifndef I2C_REG_H
#define I2C_REG_H

/*** I2C registers ***/

typedef struct {
	volatile unsigned int CR1;    		// I2C control register 1.
	volatile unsigned int CR2;    		// I2C control register 2.
	volatile unsigned int OAR1;    		// I2C own address 1 register.
	volatile unsigned int OAR2;    		// I2C own address 2 register.
	volatile unsigned int TIMINGR;  	// I2C timing register.
	volatile unsigned int TIMEOUTR;		// I2C timeout register.
	volatile unsigned int ISR;    		// I2C interrupt and status register.
	volatile unsigned int ICR;    		// I2C interrupt clear register.
	volatile unsigned int PECR;    		// I2C PEC register.
	volatile unsigned int RXDR;    		// I2C receive data register.
	volatile unsigned int TXDR;    		// I2C transmit data register.
} I2C_BaseAddress;

/*** I2C base addresses ***/

#define I2C1	((I2C_BaseAddress*) ((unsigned int) 0x40005400))
//#define I2C2	((I2C_BaseAddress*) ((unsigned int) 0x40005800))
//#define I2C3	((I2C_BaseAddress*) ((unsigned int) 0x40007800))

#endif /* I2C_REG_H */
