/*
 * rcc.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#ifndef RCC_H
#define RCC_H

/*** RCC macros ***/

#define RCC_LSI_FREQUENCY_HZ	38000
#define RCC_LSE_FREQUENCY_HZ	32768

/*** RCC functions ***/

void RCC_Init(void);
void RCC_EnableGpio(void);
void RCC_DisableGpio(void);
unsigned int RCC_GetSysclkKhz(void);
unsigned char RCC_SwitchToMsi(void);
unsigned char RCC_SwitchToHsi(void);
unsigned char RCC_SwitchToHse(void);
unsigned char RCC_EnableLsi(void);
unsigned int RCC_GetLsiFrequency(void);
unsigned char RCC_EnableLse(void);

#endif /* RCC_H */
