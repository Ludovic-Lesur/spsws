/*
 * iwdg_reg.h
 *
 *  Created on: 30 mar. 2018
 *      Author: Ludo
 */

#ifndef __IWDG_REG_H__
#define __IWDG_REG_H__

#include "types.h"

/*** IWDG REG macros ***/

// Peripheral base address.
#define IWDG	((IWDG_registers_t*) ((uint32_t) 0x40003000))

/*** IWDG REG structures ***/

/*!******************************************************************
 * \enum IWDG_registers_t
 * \brief IWDG registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t KR;    	// IWDG key register.
	volatile uint32_t PR;   	// IWDG prescaler register.
	volatile uint32_t RLR;  	// IWDG reload register.
	volatile uint32_t SR;    	// IWDG status register.
	volatile uint32_t WINR;		// IWDG window register.
} IWDG_registers_t;

#endif /* __IWDG_REG_H___ */
