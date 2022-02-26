/*
 * scb_reg.h
 *
 *  Created on: 5 may 2018
 *      Author: Ludo
 */

#ifndef SCB_REG_H
#define SCB_REG_H

/*** SCB registers ***/

typedef struct {
	volatile unsigned int CPUID;	// SCB CPUID register.
	volatile unsigned int ICSR;		// SCB interrupt control and state register.
	volatile unsigned int VTOR;		// SCB vector table offset register.
	volatile unsigned int AIRCR;	// SCB application and reset control register.
	volatile unsigned int SCR;		// SCB system control register.
	volatile unsigned int CCR;		// SCB configuration and control register.
	volatile unsigned int SHPR2;	// SCB system handler priority register 2.
	volatile unsigned int SHPR3;	// SCB system handler priority register 3.
} SCB_base_address_t;

/*** SCB base address ***/

#define SCB		((SCB_base_address_t*) ((unsigned int) 0xE000ED00))

#endif /* SCB_REG_H */
