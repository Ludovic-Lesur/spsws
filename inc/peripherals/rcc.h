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
unsigned char RCC_SwitchToInternal16MHz(void);
unsigned char RCC_SwitchToTcxo16MHz(void);
unsigned char RCC_SwitchToInternal32kHz(void);
unsigned char RCC_SwitchToQuartz32kHz(void);

#endif /* RCC_H */
