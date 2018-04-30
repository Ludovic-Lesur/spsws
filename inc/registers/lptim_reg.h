/*
 * lptim_reg.h
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludovic
 */

#ifndef REGISTERS_LPTIM_REG_H
#define REGISTERS_LPTIM_REG_H

/*** LPTIM registers ***/

typedef struct {
	volatile unsigned int ISR;    	// LPTIM interrupt and status register.
	volatile unsigned int ICR;   	// LPTIM interrupt clear register.
	volatile unsigned int IER;  	// LPTIM interrupt enable register.
	volatile unsigned int CFGR;    	// LPTIM configuration register.
	volatile unsigned int CR;      	// LPTIM control register.
	volatile unsigned int CMP;      // LPTIM compare register.
	volatile unsigned int ARR;    	// LPTIM autoreload register.
	volatile unsigned int CNT;     	// LPTIM counter register.
} LPTIM_BaseAddress;

/*** LPTIM base address ***/

#define LPTIM1	((LPTIM_BaseAddress*) ((unsigned int) 0x40007C00))

#endif /* REGISTERS_LPTIM_REG_H */
