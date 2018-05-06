/*
 * pwr_reg.h
 *
 *  Created on: 5 may 2018
 *      Author: Ludovic
 */

#ifndef REGISTERS_PWR_REG_H
#define REGISTERS_PWR_REG_H

/*** PWR registers ***/

typedef struct {
	volatile unsigned int CR;	// Power control register.
	volatile unsigned int CSR;	// Power control and status register.
} PWR_BaseAddress;

/*** PWR base address ***/

#define PWR		((PWR_BaseAddress*) ((unsigned int) 0x40007000))

#endif /* REGISTERS_PWR_REG_H */
