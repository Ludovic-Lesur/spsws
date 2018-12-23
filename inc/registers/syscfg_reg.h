/*
 * syscfg_reg.h
 *
 *  Created on: 18 june 2018
 *      Author: Ludo
 */

#ifndef SYSCFG_REG_H_
#define SYSCFG_REG_H_

/*** SYSCFG registers ***/

typedef struct {
	volatile unsigned int CFGR1;    	// SYSCFG memory remap register.
	volatile unsigned int CFGR2;    	// SYSCFG peripheral mode configuration register.
	volatile unsigned int EXTICR1;  	// SYSCFG external interrupt configuration register 1.
	volatile unsigned int EXTICR2;  	// SYSCFG external interrupt configuration register 2.
	volatile unsigned int EXTICR3;  	// SYSCFG external interrupt configuration register 3.
	volatile unsigned int EXTICR4;  	// SYSCFG external interrupt configuration register 4.
	volatile unsigned int COMP1_CSR;   	// SYSCFG comparator 1 control and status register.
	volatile unsigned int COMP2_CSR;   	// SYSCFG comparator 2 control and status register.
	volatile unsigned int CFGR3;   		// SYSCFG control and status register.
} SYSCFG_BaseAddress;

/*** SYSCFG base address ***/

#define SYSCFG	((SYSCFG_BaseAddress*) ((unsigned int) 0x40010000))

#endif /* SYSCFG_REG_H_ */
