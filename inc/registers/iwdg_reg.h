/*
 * iwdg_reg.h
 *
 *  Created on: 30 march 2018
 *      Author: Ludo
 */

#ifndef __IWDG_REG_H__
#define __IWDG_REG_H__

#include "types.h"

/*** IWDG registers ***/

typedef struct {
	volatile uint32_t KR;    	// IWDG key register.
	volatile uint32_t PR;   	// IWDG prescaler register.
	volatile uint32_t RLR;  	// IWDG reload register.
	volatile uint32_t SR;    	// IWDG status register.
	volatile uint32_t WINR;		// IWDG window register.
} IWDG_base_address_t;

/*** IWDG base address ***/

#define IWDG	((IWDG_base_address_t*) ((uint32_t) 0x40003000))

#endif /* __IWDG_REG_H___ */
