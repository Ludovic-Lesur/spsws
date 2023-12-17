/*
 * syscfg_reg.h
 *
 *  Created on: 18 jun. 2018
 *      Author: Ludo
 */

#ifndef __SYSCFG_REG_H__
#define __SYSCFG_REG_H__

#include "types.h"

/*** SYSCFG REG macros ***/

// Peripheral base address.
#define SYSCFG	((SYSCFG_registers_t*) ((uint32_t) 0x40010000))

/*** SYSCFG REG structures ***/

/*!******************************************************************
 * \enum SYSCFG_registers_t
 * \brief SYSCFG registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t CFGR1;    	// SYSCFG memory remap register.
	volatile uint32_t CFGR2;    	// SYSCFG peripheral mode configuration register.
	volatile uint32_t EXTICR[4];  	// SYSCFG external interrupt configuration registers 1-4.
	volatile uint32_t COMP1_CSR;   	// SYSCFG comparator 1 control and status register.
	volatile uint32_t COMP2_CSR;   	// SYSCFG comparator 2 control and status register.
	volatile uint32_t CFGR3;   		// SYSCFG control and status register.
} SYSCFG_registers_t;

#endif /* __SYSCFG_REG_H__ */
