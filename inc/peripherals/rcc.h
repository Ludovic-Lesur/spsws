/*
 * rcc.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_RCC_H
#define PERIPHERALS_RCC_H

/*** RCC macros ***/

//#define USE_HSE	// To be defined if external oscillator is used.

/*** RCC functions ***/

void RCC_Init(void);
unsigned int RCC_GetSysclkKhz(void);
void RCC_SwitchToMsi131kHz(void);
void RCC_SwitchToHsi16MHz(void);
void RCC_SwitchToHse(void);

#endif /* PERIPHERALS_RCC_H */
