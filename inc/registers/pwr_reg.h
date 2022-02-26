/*
 * pwr_reg.h
 *
 *  Created on: 5 may 2018
 *      Author: Ludo
 */

#ifndef PWR_REG_H
#define PWR_REG_H

/*** PWR registers ***/

typedef struct {
	volatile unsigned int CR;	// Power control register.
	volatile unsigned int CSR;	// Power control and status register.
} PWR_base_address_t;

/*** PWR base address ***/

#define PWR		((PWR_base_address_t*) ((unsigned int) 0x40007000))

#endif /* PWR_REG_H */
