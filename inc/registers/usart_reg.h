/*
 * usart_reg.h
 *
 *  Created on: 11 aug 2018
 *      Author: Ludovic
 */

#ifndef REGISTERS_USART_REG_H_
#define REGISTERS_USART_REG_H_

/*** USART registers ***/

typedef struct {
	volatile unsigned int CR1;    	// USART control register 1.
	volatile unsigned int CR2;   	// USART control register 2.
	volatile unsigned int CR3;  	// USART control register 3.
	volatile unsigned int BRR;    	// USART baud rate register.
	volatile unsigned int GTPR;		// USART guard time and prescaler register.
	volatile unsigned int RTOR;		// USART receiver timeout register.
	volatile unsigned int RQR;      // USART request register.
	volatile unsigned int ISR;      // USART interrupt and status register.
	volatile unsigned int ICR;    	// USART interrupt flag clear register.
	volatile unsigned int RDR;     	// USART receive data register.
	volatile unsigned int TDR;   	// USART transmit data register.
} USART_BaseAddress;

/*** USART base address ***/

#define USART2	((USART_BaseAddress*) ((unsigned int) 0x40004400))

#endif /* REGISTERS_USART_REG_H_ */
