/*
 * nvic.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#ifndef NVIC_H
#define NVIC_H

/*** NVIC interrupts vector ***/

typedef enum {
	IT_WWDG = 0,
	IT_PVD = 1,
	IT_RTC = 2,
	IT_FLASH = 3,
	IT_RCC_CRS = 4,
	IT_EXTI_0_1 = 5,
	IT_EXTI_2_3 = 6,
	IT_EXTI_4_15 = 7,
	IT_RESERVED0 = 8,
	IT_DMA1_Channel1 = 9,
	IT_DMA1_Channel2_3 = 10,
	IT_DMA1_Channel4_7 = 11,
	IT_ADC_COMP = 12,
	IT_LPTIM1 = 13,
	IT_USART4_5 = 14,
	IT_TIM2 = 15,
	IT_TIM3 = 16,
	IT_TIM6 = 17,
	IT_TIM7 = 18,
	IT_RESERVED1 = 19,
	IT_TIM21 = 20,
	IT_I2C3 = 21,
	IT_TIM22 = 22,
	IT_I2C1 = 23,
	IT_I2C2 = 24,
	IT_SPI1 = 25,
	IT_SPI2 = 26,
	IT_USART1 = 27,
	IT_USART2 = 28,
	IT_LPUART1 = 29
} InterruptVector;

/*** NVIC functions ***/

void NVIC_EnableInterrupt(InterruptVector it_num);
void NVIC_DisableInterrupt(InterruptVector it_num);
void NVIC_SetPriority(InterruptVector it_num, unsigned char priority);

#endif /* NVIC_H */
