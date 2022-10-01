/*
 * usart_reg.h
 *
 *  Created on: 11 aug 2018
 *      Author: Ludo
 */

#ifndef __USART_REG_H__
#define __USART_REG_H__

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
} USART_base_address_t;

/*** USART base address ***/

#ifdef HW2_0
#define USART1	((USART_base_address_t*) ((unsigned int) 0x40013800))
#endif
#define USART2	((USART_base_address_t*) ((unsigned int) 0x40004400))

#endif /* __USART_REG_H__ */
