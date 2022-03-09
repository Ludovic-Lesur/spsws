/*
 * rcc.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#ifndef RCC_H
#define RCC_H

#include "flash.h"
#include "tim.h"

/*** RCC macros ***/

#define RCC_LSI_FREQUENCY_HZ	38000
#define RCC_LSE_FREQUENCY_HZ	32768
#define RCC_MSI_FREQUENCY_KHZ	65
#define RCC_HSI_FREQUENCY_KHZ	16000
#define RCC_TCXO_FREQUENCY_KHZ	16000

/*** RCC structures ***/

typedef enum {
	RCC_SUCCESS = 0,
	RCC_ERROR_MSI_READY,
	RCC_ERROR_MSI_SWITCH,
	RCC_ERROR_HSI_READY,
	RCC_ERROR_HSI_SWITCH,
	RCC_ERROR_HSE_READY,
	RCC_ERROR_HSE_SWITCH,
	RCC_ERROR_LSI_READY,
	RCC_ERROR_LSI_MEASUREMENT,
	RCC_ERROR_LSE_READY,
	RCC_ERROR_LAST,
	RCC_ERROR_BASE_FLASH = 0x0100,
	RCC_ERROR_BASE_TIM = (RCC_ERROR_BASE_FLASH + FLASH_ERROR_BASE_LAST),
	RCC_ERROR_BASE_LAST = (RCC_ERROR_BASE_TIM + TIM_ERROR_BASE_LAST)
} RCC_status_t;

/*** RCC functions ***/

void RCC_init(void);
void RCC_enable_gpio(void);
void RCC_disable_gpio(void);
RCC_status_t RCC_switch_to_msi(void);
RCC_status_t RCC_switch_to_hsi(void);
RCC_status_t RCC_switch_to_hse(void);
unsigned int RCC_get_sysclk_khz(void);
RCC_status_t RCC_enable_lsi(void);
RCC_status_t RCC_get_lsi_frequency(unsigned int* lsi_frequency_hz);
RCC_status_t RCC_enable_lse(void);

#define RCC_status_check(error_base) { if (rcc_status != RCC_SUCCESS) { status = error_base + rcc_status; goto errors; }}

#endif /* RCC_H */
