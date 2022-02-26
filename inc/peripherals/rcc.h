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
#define RCC_MSI_FREQUENCY_KHZ	65
#define RCC_HSI_FREQUENCY_KHZ	16000
#define RCC_TCXO_FREQUENCY_KHZ	16000

/*** RCC functions ***/

void RCC_init(void);
void RCC_enable_gpio(void);
void RCC_disable_gpio(void);
unsigned int RCC_get_sysclk_khz(void);
unsigned char RCC_switch_to_msi(void);
unsigned char RCC_switch_to_hsi(void);
unsigned char RCC_switch_to_hse(void);
unsigned char RCC_enable_lsi(void);
void RCC_get_lsi_frequency(unsigned int* lsi_frequency_hz);
unsigned char RCC_enable_lse(void);

#endif /* RCC_H */
