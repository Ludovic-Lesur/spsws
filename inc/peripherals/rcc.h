/*
 * rcc.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#ifndef RCC_H
#define RCC_H

/*** RCC macros ***/

// System clock frequency in kHz.
#define SYSCLK_KHZ	16000
// To be defined if external oscillator is used.
//#define USE_HSE

/*** RCC functions ***/

void RCC_Init(void);
void RCC_SwitchToInternal16MHz(void);
void RCC_SwitchToTcxo16MHz(void);

#endif /* RCC_H */
