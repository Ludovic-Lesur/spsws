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
	volatile unsigned int EXTICR[4];  	// SYSCFG external interrupt configuration registers 1-4.
	volatile unsigned int COMP1_CSR;   	// SYSCFG comparator 1 control and status register.
	volatile unsigned int COMP2_CSR;   	// SYSCFG comparator 2 control and status register.
	volatile unsigned int CFGR3;   		// SYSCFG control and status register.
} SYSCFG_base_address_t;

/*** SYSCFG base address ***/

#define SYSCFG	((SYSCFG_base_address_t*) ((unsigned int) 0x40010000))

#endif /* SYSCFG_REG_H_ */
