/*
 * tim_reg.h
 *
 *  Created on: 03 may 2018
 *      Author: Ludo
 */

#ifndef __TIM_REG_H__
#define __TIM_REG_H__

#include "types.h"

/*** TIM REG macros ***/

// Peripherals base address.
#define TIM2	((TIM_registers_t*) ((uint32_t) 0x40000000))
#define TIM21	((TIM_registers_t*) ((uint32_t) 0x40010800))
#if (defined MCU_CATEGORY_2) || (defined MCU_CATEGORY_3) || (defined MCU_CATEGORY_5)
#define TIM22	((TIM_registers_t*) ((uint32_t) 0x40011400))
#endif
#if (defined MCU_CATEGORY_3) || (defined MCU_CATEGORY_5)
#define TIM6	((TIM_registers_t*) ((uint32_t) 0x40001000))
#endif
#ifdef MCU_CATEGORY_5
#define TIM3	((TIM_registers_t*) ((uint32_t) 0x40000400))
#define TIM7	((TIM_registers_t*) ((uint32_t) 0x40001400))
#endif

/*** TIM REG structures ***/

/*!******************************************************************
 * \enum TIM_registers_t
 * \brief TIM registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t CR1;    			// Control register 1.
	volatile uint32_t CR2;    			// Control register 2.
	volatile uint32_t SMCR;    			// Slave mode controler register.
	volatile uint32_t DIER;    			// DMA interrupt enable register.
	volatile uint32_t SR;    			// Status register.
	volatile uint32_t EGR;    			// Event generation register.
	volatile uint32_t CCMR1;    		// Capture/compare mode register 1.
	volatile uint32_t CCMR2;    		// Capture/compare mode register 2.
	volatile uint32_t CCER;    			// Capture/compare enable register.
	volatile uint32_t CNT;    			// Counter register.
	volatile uint32_t PSC;    			// Prescaler register.
	volatile uint32_t ARR;    			// Auto-reload register.
	volatile uint32_t RESERVED0;		// Reserved 0x30.
	union {
		struct {
			volatile uint32_t CCR1;		// Capture/compare register 1.
			volatile uint32_t CCR2;		// Capture/compare register 2.
			volatile uint32_t CCR3;		// Capture/compare register 3.
			volatile uint32_t CCR4;		// Capture/compare register 4.
		};
		volatile uint32_t CCRx[4];		// Capture/compare registers.
	};
	volatile uint32_t RESERVED1;    	// Reserved 0x44
	volatile uint32_t DCR;    			// DMA control register.
	volatile uint32_t DMAR;    			// DMA address for full transfer register.
	volatile uint32_t OR;    			// Option register.
} TIM_registers_t;

#endif /* __TIM_REG_H__ */
