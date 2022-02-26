/*
 * exti_reg.h
 *
 *  Created on: 18 june 2018
 *      Author: Ludo
 */

#ifndef EXTI_REG_H_
#define EXTI_REG_H_

/*** EXTI registers ***/

typedef struct {
	volatile unsigned int IMR;    	// EXTI interrupt mask register.
	volatile unsigned int EMR;    	// EXTI event mask register.
	volatile unsigned int RTSR;    	// EXTI rising edge trigger selection register.
	volatile unsigned int FTSR;    	// EXTI falling edge trigger selection register.
	volatile unsigned int SWIER;    // EXTI software interrupt event register.
	volatile unsigned int PR;    	// EXTI pending register.
} EXTI_base_address_t;

/*** EXTI base address ***/

#define EXTI	((EXTI_base_address_t*) ((unsigned int) 0x40010400))

#endif /* EXTI_REG_H_ */
