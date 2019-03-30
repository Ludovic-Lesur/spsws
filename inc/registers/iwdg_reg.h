/*
 * iwdg_reg.h
 *
 *  Created on: 30 march 2018
 *      Author: Ludovic
 */

#ifndef IWDG_REG_H
#define IWDG_REG_H

/*** IWDG registers ***/

typedef struct {
	volatile unsigned int KR;    	// IWDG key register.
	volatile unsigned int PR;   	// IWDG prescaler register.
	volatile unsigned int RLR;  	// IWDG reload register.
	volatile unsigned int SR;    	// IWDG status register.
	volatile unsigned int WINR;		// IWDG window register.
} IWDG_BaseAddress;

/*** IWDG base address ***/

#define IWDG	((IWDG_BaseAddress*) ((unsigned int) 0x40003000))

#endif /* IWDG_REG_H_ */
