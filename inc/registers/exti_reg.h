/*
 * exti_reg.h
 *
 *  Created on: 18 june 2018
 *      Author: Ludovic
 */

#ifndef REGISTERS_EXTI_REG_H_
#define REGISTERS_EXTI_REG_H_

/*** EXTI registers ***/

typedef struct {
	volatile unsigned int IMR;    	// EXTI interrupt mask register.
	volatile unsigned int EMR;    	// EXTI event mask register.
	volatile unsigned int RTSR;    	// EXTI rising edge trigger selection register.
	volatile unsigned int FTSR;    	// EXTI falling edge trigger selection register.
	volatile unsigned int SWIER;    // EXTI software interrupt event register.
	volatile unsigned int PR;    	// EXTI pending register.
} EXTI_BaseAddress;

/*** EXTI base address ***/

#define EXTI	((EXTI_BaseAddress*) ((unsigned int) 0x40010400))

#endif /* REGISTERS_EXTI_REG_H_ */
