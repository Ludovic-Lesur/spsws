/*
 * pwr_reg.h
 *
 *  Created on: 5 may 2018
 *      Author: Ludo
 */

#ifndef __PWR_REG_H__
#define __PWR_REG_H__

#include "types.h"

/*** PWR registers ***/

typedef struct {
	volatile uint32_t CR;	// Power control register.
	volatile uint32_t CSR;	// Power control and status register.
} PWR_registers_t;

/*** PWR base address ***/

#define PWR		((PWR_registers_t*) ((uint32_t) 0x40007000))

#endif /* __PWR_REG_H__ */
