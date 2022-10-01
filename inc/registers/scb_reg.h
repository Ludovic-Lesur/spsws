/*
 * scb_reg.h
 *
 *  Created on: 5 may 2018
 *      Author: Ludo
 */

#ifndef __SCB_REG_H__
#define __SCB_REG_H__

#include "types.h"

/*** SCB registers ***/

typedef struct {
	volatile uint32_t CPUID;	// SCB CPUID register.
	volatile uint32_t ICSR;		// SCB interrupt control and state register.
	volatile uint32_t VTOR;		// SCB vector table offset register.
	volatile uint32_t AIRCR;	// SCB application and reset control register.
	volatile uint32_t SCR;		// SCB system control register.
	volatile uint32_t CCR;		// SCB configuration and control register.
	volatile uint32_t SHPR2;	// SCB system handler priority register 2.
	volatile uint32_t SHPR3;	// SCB system handler priority register 3.
} SCB_base_address_t;

/*** SCB base address ***/

#define SCB		((SCB_base_address_t*) ((uint32_t) 0xE000ED00))

#endif /* __SCB_REG_H__ */
