/*
 * rcc_reg.h
 *
 *  Created on: 26 apr. 2018
 *      Author: Ludo
 */

#ifndef __RCC_REG_H__
#define __RCC_REG_H__

#include "types.h"

/*** RCC REG macros ***/

// Peripheral base address.
#define RCC		((RCC_registers_t*) ((uint32_t) 0x40021000))

/*** RCC REG structures ***/

/*!******************************************************************
 * \enum RCC_registers_t
 * \brief RCC registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t CR;			// RCC clock control register.
	volatile uint32_t ICSCR;		// RCC internal clock sources calibration register.
	volatile uint32_t RESERVED0;	// Reserved 0x08.
	volatile uint32_t CFGR;			// RCC clock configuration register.
	volatile uint32_t CIER;			// RCC clock interrupt enable register.
	volatile uint32_t CIFR;			// RCC clock interrupt flag register.
	volatile uint32_t CICR;			// RCC clock interrupt clear register.
	volatile uint32_t IOPRSTR;		// RCC GPIO reset register.
	volatile uint32_t AHBRSTR;		// RCC AHB peripheral reset register.
	volatile uint32_t APB2RSTR;		// RCC APB2 peripheral reset register.
	volatile uint32_t APB1RSTR;		// RCC APB1 peripheral reset register.
	volatile uint32_t IOPENR;		// RCC GPIO clock enable register.
	volatile uint32_t AHBENR;		// RCC AHB peripheral clock enable register.
	volatile uint32_t APB2ENR;		// RCC APB2 peripheral clock enable register.
	volatile uint32_t APB1ENR;		// RCC APB1 peripheral clock enable register.
	volatile uint32_t IOPSMENR;		// RCC GPIO clock enable in sleep mode register.
	volatile uint32_t AHBSMENR;		// RCC AHB peripheral clock enable in sleep mode register.
	volatile uint32_t APB2SMENR;	// RCC APB2 peripheral clock enable in sleep mode register.
	volatile uint32_t APB1SMENR;	// RCC APB1 peripheral clock enable in sleep mode register.
	volatile uint32_t CCIPR;		// RCC clock configuration register.
	volatile uint32_t CSR;			// RCC control and status register.
} RCC_registers_t;

#endif /* __RCC_REG_H__ */
