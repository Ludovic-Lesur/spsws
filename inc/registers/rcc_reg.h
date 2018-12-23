/*
 * rcc_reg.h
 *
 *  Created on: 26 apr. 2018
 *      Author: Ludo
 */

#ifndef RCC_REG_H
#define RCC_REG_H

/*** RCC registers ***/

typedef struct {
	volatile unsigned int CR;			// RCC clock control register.
	volatile unsigned int ICSCR;		// RCC internal clock sources calibration register.
	unsigned int RESERVED0;				// Reserved 0x08.
	volatile unsigned int CFGR;			// RCC clock configuration register.
	volatile unsigned int CIER;			// RCC clock interrupt enable register.
	volatile unsigned int CIFR;			// RCC clock interrupt flag register.
	volatile unsigned int CICR;			// RCC clock interrupt clear register.
	volatile unsigned int IOPRSTR;		// RCC GPIO reset register.
	volatile unsigned int AHBRSTR;		// RCC AHB peripheral reset register.
	volatile unsigned int APB2RSTR;		// RCC APB2 peripheral reset register.
	volatile unsigned int APB1RSTR;		// RCC APB1 peripheral reset register.
	volatile unsigned int IOPENR;		// RCC GPIO clock enable register.
	volatile unsigned int AHBENR;		// RCC AHB peripheral clock enable register.
	volatile unsigned int APB2ENR;		// RCC APB2 peripheral clock enable register.
	volatile unsigned int APB1ENR;		// RCC APB1 peripheral clock enable register.
	volatile unsigned int IOPSMENR;		// RCC GPIO clock enable in sleep mode register.
	volatile unsigned int AHBSMENR;		// RCC AHB peripheral clock enable in sleep mode register.
	volatile unsigned int APB2SMENR;	// RCC APB2 peripheral clock enable in sleep mode register.
	volatile unsigned int APB1SMENR;	// RCC APB1 peripheral clock enable in sleep mode register.
	volatile unsigned int CCIPR;		// RCC clock configuration register.
	volatile unsigned int CSR;			// RCC control and status register.
} RCC_BaseAddress;

/*** RCC base address ***/

#define RCC		((RCC_BaseAddress*) ((unsigned int) 0x40021000))

#endif /* RCC_REG_H */
