/*
 * nvic_reg.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#ifndef __NVIC_REG_H__
#define __NVIC_REG_H__

#include "types.h"

/*** NVIC REG macros ***/

// Peripheral base address.
#define NVIC	((NVIC_registers_t*) ((uint32_t) 0xE000E100))

/*** NVIC REG structures ***/

/*!******************************************************************
 * \enum NVIC_registers_t
 * \brief NVIC registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t ISER;				// Interrupt set-enable register.
	volatile uint32_t RESERVED0[31];	// Reserved 0xE000E104.
	volatile uint32_t ICER;				// Interrupt clear-enable register.
	volatile uint32_t RESERVED1[31];	// Reserved 0xE000E184.
	volatile uint32_t ISPR;				// Interrupt set-pending register.
	volatile uint32_t RESERVED2[31];	// Reserved 0xE000E204.
	volatile uint32_t ICPR;    			// Interrupt clear-pending register.
	volatile uint32_t RESERVED3[95];	// Reserved 0xE000E300.
	volatile uint32_t IPR[8];			// Interrupt priority registers 0 to 7.
} NVIC_registers_t;

#endif /* __NVIC_REG_H__ */
