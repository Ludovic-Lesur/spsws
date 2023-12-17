/*
 * scb_reg.h
 *
 *  Created on: 05 may 2018
 *      Author: Ludo
 */

#ifndef __SCB_REG_H__
#define __SCB_REG_H__

#include "types.h"

/*** SCB REG macros ***/

// Peripheral base address.
#define SCB		((SCB_registers_t*) ((uint32_t) 0xE000ED00))

/*** SCB REG structures ***/

/*!******************************************************************
 * \enum SCB_registers_t
 * \brief SCB registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t CPUID;	// SCB CPUID register.
	volatile uint32_t ICSR;		// SCB interrupt control and state register.
	volatile uint32_t VTOR;		// SCB vector table offset register.
	volatile uint32_t AIRCR;	// SCB application and reset control register.
	volatile uint32_t SCR;		// SCB system control register.
	volatile uint32_t CCR;		// SCB configuration and control register.
	volatile uint32_t SHPR2;	// SCB system handler priority register 2.
	volatile uint32_t SHPR3;	// SCB system handler priority register 3.
} SCB_registers_t;

#endif /* __SCB_REG_H__ */
