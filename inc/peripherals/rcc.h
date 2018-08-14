/*
 * rcc.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_RCC_H
#define PERIPHERALS_RCC_H

/*** RCC macros ***/

// System clock frequency in kHz.
#define SYSCLK_KHZ	16000
// To be defined if external oscillator is used.
//#define USE_HSE

/*** RCC functions ***/

void RCC_Init(void);
void RCC_SwitchToHsi16MHz(void);
void RCC_SwitchToHse16MHz(void);

#endif /* PERIPHERALS_RCC_H */
