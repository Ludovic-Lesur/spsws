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
#define RCC_SYSCLK_KHZ		16000

/*** RCC functions ***/

void RCC_Init(void);
unsigned char RCC_SwitchToHsi(void);
unsigned char RCC_SwitchToHse(void);
unsigned char RCC_EnableLsi(void);
unsigned int RCC_GetLsiFrequency(void);
unsigned char RCC_EnableLse(void);

#endif /* RCC_H */
