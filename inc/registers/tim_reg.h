/*
 * tim_reg.h
 *
 *  Created on: 3 may 2018
 *      Author: Ludo
 */

#ifndef __TIM_REG_H__
#define __TIM_REG_H__

#include "types.h"

/*** TIMx registers ***/

typedef struct {
	volatile uint32_t CR1;    		// Control register 1.
	volatile uint32_t CR2;    		// Control register 2.
	volatile uint32_t SMCR;    		// Slave mode controler register (!)
	volatile uint32_t DIER;    		// DMA interrupt enable register.
	volatile uint32_t SR;    		// Status register.
	volatile uint32_t EGR;    		// Event generation register.
	volatile uint32_t CCMR1;    	// Capture/compare mode register 1 (!).
	volatile uint32_t CCMR2;    	// Capture/compare mode register 2 (!).
	volatile uint32_t CCER;    		// Capture/compare enable register (!).
	volatile uint32_t CNT;    		// Counter register.
	volatile uint32_t PSC;    		// Prescaler register.
	volatile uint32_t ARR;    		// Auto-reload register.
	volatile uint32_t RESERVED0;	// Reserved 0x30.
	volatile uint32_t CCR1;    		// Capture/compare register 1 (!).
	volatile uint32_t CCR2;    		// Capture/compare register 2 (!).
	volatile uint32_t CCR3;    		// Capture/compare register 3 (!).
	volatile uint32_t CCR4;    		// Capture/compare register 4 (!).
	volatile uint32_t RESERVED1;    // Reserved 0x44
	volatile uint32_t DCR;    		// DMA control register (!).
	volatile uint32_t DMAR;    		// DMA address for full transfer register (!).
	volatile uint32_t OR;    		// Option register (!).
} TIM_base_address_t;

/*** TIMx base addresses ***/

#define TIM2	((TIM_base_address_t*) ((uint32_t) 0x40000000))
#ifdef HW2_0
#define TIM3	((TIM_base_address_t*) ((uint32_t) 0x40000400)) // Not present on STM32L041K6xx.
#endif
#define TIM21	((TIM_base_address_t*) ((uint32_t) 0x40010800))
#define TIM22	((TIM_base_address_t*) ((uint32_t) 0x40011400))
#ifdef HW2_0
#define TIM6	((TIM_base_address_t*) ((uint32_t) 0x40001000)) // Not present on STM32L041K6xx.
#define TIM7	((TIM_base_address_t*) ((uint32_t) 0x40001400)) // Not present on STM32L041K6xx.
#endif

#endif /* __TIM_REG_H__ */
