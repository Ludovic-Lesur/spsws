/*
 * rcc.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_RCC_H
#define PERIPHERALS_RCC_H

/*** RCC functions ***/

//#define USE_HSE	// To be defined if external oscillator is used.

// See RCC_Init() function for peripherals clock prescalers settings.
#ifdef USE_HSE
#define SYSCLK_KHZ	16000
#else
#define SYSCLK_KHZ	16000
#endif

/*** RCC functions ***/

void RCC_Init(void);
void RCC_SwitchToMsi65kHz(void);
void RCC_SwitchToHsi16MHz(void);
void RCC_SwitchToHse(void);

#endif /* PERIPHERALS_RCC_H */
