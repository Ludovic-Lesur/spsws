/*
 * nvic.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#ifndef __NVIC_H__
#define __NVIC_H__

#include "types.h"

/*** NVIC interrupts vector ***/

typedef enum {
	NVIC_IT_WWDG = 0,
	NVIC_IT_PVD = 1,
	NVIC_IT_RTC = 2,
	NVIC_IT_FLASH = 3,
	NVIC_IT_RCC_CRS = 4,
	NVIC_IT_EXTI_0_1 = 5,
	NVIC_IT_EXTI_2_3 = 6,
	NVIC_IT_EXTI_4_15 = 7,
	NVIC_IT_RESERVED0 = 8,
	NVIC_IT_DMA1_CHA1 = 9,
	NVIC_IT_DMA1_CH_2_3 = 10,
	NVIC_IT_DMA1_CH_4_7 = 11,
	NVIC_IT_ADC_COMP = 12,
	NVIC_IT_LPTIM1 = 13,
	NVIC_IT_USART4_USART5 = 14,
	NVIC_IT_TIM2 = 15,
	NVIC_IT_TIM3 = 16,
	NVIC_IT_TIM6 = 17,
	NVIC_IT_TIM7 = 18,
	NVIC_IT_RESERVED1 = 19,
	NVIC_IT_TIM21 = 20,
	NVIC_IT_I2C3 = 21,
	NVIC_IT_TIM22 = 22,
	NVIC_IT_I2C1 = 23,
	NVIC_IT_I2C2 = 24,
	NVIC_IT_SPI1 = 25,
	NVIC_IT_SPI2 = 26,
	NVIC_IT_USART1 = 27,
	NVIC_IT_USART2 = 28,
	NVIC_IT_LPUART1 = 29,
	NVIC_IT_LAST
} NVIC_interrupt_t;

typedef enum {
	NVIC_PRIORITY_MAX = 0,
	NVIC_PRIORITY_MIN = 3,
	NVIC_PRIORITY_LAST
} NVIC_priority_t;

/*** NVIC functions ***/

void NVIC_init(void);
void NVIC_enable_interrupt(NVIC_interrupt_t it_num);
void NVIC_disable_interrupt(NVIC_interrupt_t it_num);
void NVIC_set_priority(NVIC_interrupt_t it_num, uint8_t priority);

#endif /* __NVIC_H__ */
