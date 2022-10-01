/*
 * nvic_reg.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#ifndef __NVIC_REG_H__
#define __NVIC_REG_H__

/*** NVIC registers ***/

typedef struct {
	volatile unsigned int ISER;			// Interrupt set-enable register.
	unsigned int RESERVED0[31];			// Reserved 0xE000E104.
	volatile unsigned int ICER;			// Interrupt clear-enable register.
	unsigned int RESERVED1[31];			// Reserved 0xE000E184.
	volatile unsigned int ISPR;			// Interrupt set-pending register.
	unsigned int RESERVED2[31];			// Reserved 0xE000E204.
	volatile unsigned int ICPR;    		// Interrupt clear-pending register.
	unsigned int RESERVED3[95];			// Reserved 0xE000E300.
	volatile unsigned int IPR[8];		// Interrupt priority registers 0 to 7.
} NVIC_base_address_t;

/*** NVIC base address ***/

#define NVIC	((NVIC_base_address_t*) ((unsigned int) 0xE000E100))

#endif /* __NVIC_REG_H__ */
