/*
 * nvic.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#ifndef __NVIC_H__
#define __NVIC_H__

#include "types.h"

/*** NVIC structures ***/

/*!******************************************************************
 * \enum NVIC_interrupt_t
 * \brief NVIC interrupt vector.
 *******************************************************************/
typedef enum {
	NVIC_INTERRUPT_WWDG = 0,
	NVIC_INTERRUPT_PVD = 1,
	NVIC_INTERRUPT_RTC = 2,
	NVIC_INTERRUPT_FLASH = 3,
	NVIC_INTERRUPT_RCC_CRS = 4,
	NVIC_INTERRUPT_EXTI_0_1 = 5,
	NVIC_INTERRUPT_EXTI_2_3 = 6,
	NVIC_INTERRUPT_EXTI_4_15 = 7,
	NVIC_INTERRUPT_RESERVED0 = 8,
	NVIC_INTERRUPT_DMA1_CHA1 = 9,
	NVIC_INTERRUPT_DMA1_CH_2_3 = 10,
	NVIC_INTERRUPT_DMA1_CH_4_7 = 11,
	NVIC_INTERRUPT_ADC_COMP = 12,
	NVIC_INTERRUPT_LPTIM1 = 13,
	NVIC_INTERRUPT_USART4_USART5 = 14,
	NVIC_INTERRUPT_TIM2 = 15,
	NVIC_INTERRUPT_TIM3 = 16,
	NVIC_INTERRUPT_TIM6 = 17,
	NVIC_INTERRUPT_TIM7 = 18,
	NVIC_INTERRUPT_RESERVED1 = 19,
	NVIC_INTERRUPT_TIM21 = 20,
	NVIC_INTERRUPT_I2C3 = 21,
	NVIC_INTERRUPT_TIM22 = 22,
	NVIC_INTERRUPT_I2C1 = 23,
	NVIC_INTERRUPT_I2C2 = 24,
	NVIC_INTERRUPT_SPI1 = 25,
	NVIC_INTERRUPT_SPI2 = 26,
	NVIC_INTERRUPT_USART1 = 27,
	NVIC_INTERRUPT_USART2 = 28,
	NVIC_INTERRUPT_LPUART1 = 29,
	NVIC_INTERRUPT_LAST
} NVIC_interrupt_t;

/*!******************************************************************
 * \enum NVIC_priority_list_t
 * \brief NVIC interrupt priorities list.
 *******************************************************************/
typedef enum {
	// Common.
	NVIC_PRIORITY_RCC_CRS = 0,
	NVIC_PRIORITY_TIM21 = 1,
	NVIC_PRIORITY_LPTIM1 = 2,
	NVIC_PRIORITY_RTC = 3,
	// GPS.
	NVIC_PRIORITY_LPUART1 = 0,
	NVIC_PRIORITY_DMA1_CH_4_7 = 1,
	// DIOs.
	NVIC_PRIORITY_EXTI_4_15 = 3,
	// Sigfox.
#ifdef HW1_0
	NVIC_PRIORITY_EXTI_2_3 = 0,
#endif
#ifdef HW2_0
	NVIC_PRIORITY_EXTI_0_1 = 0,
#endif
	NVIC_PRIORITY_TIM22 = 0,
	NVIC_PRIORITY_TIM2 = 1,
	// AT interface.
#ifdef HW1_0
	NVIC_PRIORITY_USART2 = 3,
#endif
#ifdef HW2_0
	NVIC_PRIORITY_USART1 = 3,
#endif
} NVIC_priority_list_t;

/*** NVIC functions ***/

/*!******************************************************************
 * \fn void NVIC_init(void)
 * \brief Init interrupts vector.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void NVIC_init(void);

/*!******************************************************************
 * \fn void NVIC_enable_interrupt(NVIC_interrupt_t irq_index, uint8_t priority)
 * \brief Enable interrupt.
 * \param[in]  	irq_index: Interrupt to enable.
 * \param[in]	priority: Interrupt priority to set.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void NVIC_enable_interrupt(NVIC_interrupt_t irq_index, uint8_t priority);

/*!******************************************************************
 * \fn void NVIC_disable_interrupt(NVIC_interrupt_t irq_index)
 * \brief Disable interrupt.
 * \param[in]  	irq_index: Interrupt to enable.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void NVIC_disable_interrupt(NVIC_interrupt_t irq_index);

#endif /* __NVIC_H__ */
