/*
 * usart_reg.h
 *
 *  Created on: 11 aug 2018
 *      Author: Ludo
 */

#ifndef __USART_REG_H__
#define __USART_REG_H__

#include "types.h"

/*** USART registers ***/

typedef struct {
	volatile uint32_t CR1;    	// USART control register 1.
	volatile uint32_t CR2;   	// USART control register 2.
	volatile uint32_t CR3;  	// USART control register 3.
	volatile uint32_t BRR;    	// USART baud rate register.
	volatile uint32_t GTPR;		// USART guard time and prescaler register.
	volatile uint32_t RTOR;		// USART receiver timeout register.
	volatile uint32_t RQR;      // USART request register.
	volatile uint32_t ISR;      // USART interrupt and status register.
	volatile uint32_t ICR;    	// USART interrupt flag clear register.
	volatile uint32_t RDR;     	// USART receive data register.
	volatile uint32_t TDR;   	// USART transmit data register.
} USART_registers_t;

/*** USART base address ***/

#define USART2	((USART_registers_t*) ((uint32_t) 0x40004400))
#if (defined MCU_CATEGORY_3) || (defined MCU_CATEGORY_5)
#define USART1	((USART_registers_t*) ((uint32_t) 0x40013800))
#endif
#ifdef MCU_CATEGORY_5
#define USART4	((USART_registers_t*) ((uint32_t) 0x40004C00))
#define USART5	((USART_registers_t*) ((uint32_t) 0x40005000))
#endif

#endif /* __USART_REG_H__ */
