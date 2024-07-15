/*
 * lpuart_reg.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#ifndef __LPUART_REG_H__
#define __LPUART_REG_H__

#include "types.h"

/*** LPUART REG macros ***/

// Peripheral base address.
#define LPUART1		((LPUART_registers_t*) ((uint32_t) 0x40004800))

/*** LPUART REG structures ***/

/*!******************************************************************
 * \enum LPUART_registers_t
 * \brief LPUART registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t CR1;    		// LPUART control register 1.
	volatile uint32_t CR2;   		// LPUART control register 2.
	volatile uint32_t CR3;  		// LPUART control register 3.
	volatile uint32_t BRR;    		// LPUART baud rate register.
	volatile uint32_t RESERVED0[2];	// Reserved 0x10-0x14.
	volatile uint32_t RQR;      	// LPUART request register.
	volatile uint32_t ISR;      	// LPUART interrupt and status register.
	volatile uint32_t ICR;    		// LPUART interrupt flag clear register.
	volatile uint32_t RDR;     		// LPUART receive data register.
	volatile uint32_t TDR;   		// LPUART transmit data register.
} LPUART_registers_t;

#endif /* __LPUART_REG_H__ */
