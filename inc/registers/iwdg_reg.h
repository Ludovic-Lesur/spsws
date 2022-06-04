/*
 * iwdg_reg.h
 *
 *  Created on: 30 march 2018
 *      Author: Ludo
 */

#ifndef __IWDG_REG_H__
#define __IWDG_REG_H__

/*** IWDG registers ***/

typedef struct {
	volatile unsigned int KR;    	// IWDG key register.
	volatile unsigned int PR;   	// IWDG prescaler register.
	volatile unsigned int RLR;  	// IWDG reload register.
	volatile unsigned int SR;    	// IWDG status register.
	volatile unsigned int WINR;		// IWDG window register.
} IWDG_base_address_t;

/*** IWDG base address ***/

#define IWDG	((IWDG_base_address_t*) ((unsigned int) 0x40003000))

#endif /* __IWDG_REG_H___ */
