/*
 * lpuart_reg.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#ifndef LPUART_REG_H
#define LPUART_REG_H

/*** LPUART registers ***/

typedef struct {
	volatile unsigned int CR1;    	// LPUART control register 1.
	volatile unsigned int CR2;   	// LPUART control register 2.
	volatile unsigned int CR3;  	// LPUART control register 3.
	volatile unsigned int BRR;    	// LPUART baud rate register.
	unsigned int RESERVED0[2];		// Reserved 0x10-0x14.
	volatile unsigned int RQR;      // LPUART request register.
	volatile unsigned int ISR;      // LPUART interrupt and status register.
	volatile unsigned int ICR;    	// LPUART interrupt flag clear register.
	volatile unsigned int RDR;     	// LPUART receive data register.
	volatile unsigned int TDR;   	// LPUART transmit data register.
} LPUART_base_address_t;

/*** LPUART base address ***/

#define LPUART1	((LPUART_base_address_t*) ((unsigned int) 0x40004800))

#endif /* LPUART_REG_H */
