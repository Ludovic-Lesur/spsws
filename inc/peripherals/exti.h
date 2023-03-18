/*
 * exti.h
 *
 *  Created on: 18 june 2018
 *      Author: Ludo
 */

#ifndef __EXTI_H__
#define __EXTI_H__

#include "gpio.h"

/*** EXTI structures ***/

typedef enum {
	EXTI_LINE_GPIO_0 = 0,
	EXTI_LINE_GPIO_1 = 1,
	EXTI_LINE_GPIO_2 = 2,
	EXTI_LINE_GPIO_3 = 3,
	EXTI_LINE_GPIO_4 = 4,
	EXTI_LINE_GPIO_5 = 5,
	EXTI_LINE_GPIO_6 = 6,
	EXTI_LINE_GPIO_7 = 7,
	EXTI_LINE_GPIO_8 = 8,
	EXTI_LINE_GPIO_9 = 9,
	EXTI_LINE_GPIO_10 = 10,
	EXTI_LINE_GPIO_11 = 11,
	EXTI_LINE_GPIO_12 = 12,
	EXTI_LINE_GPIO_13 = 13,
	EXTI_LINE_GPIO_14 = 14,
	EXTI_LINE_GPIO_15 = 15,
	EXTI_LINE_PVD = 16,
	EXTI_LINE_RTC_ALARM = 17,
	EXTI_LINE_RTC_TAMPER_TIMESTAMP = 19,
	EXTI_LINE_RTC_WAKEUP_TIMER = 20,
	EXTI_LINE_COMP1 = 21,
	EXTI_LINE_COMP2 = 22,
	EXTI_LINE_I2C1 = 23,
	EXTI_LINE_I2C3 = 24,
	EXTI_LINE_USART1 = 25,
	EXTI_LINE_USART2 = 26,
	EXTI_LINE_LPUART1 = 28,
	EXTI_LINE_LPTIM1 = 29,
	EXTI_LINE_LAST
} EXTI_line_t;

typedef enum {
	EXTI_TRIGGER_RISING_EDGE,
	EXTI_TRIGGER_FALLING_EDGE,
	EXTI_TRIGGER_ANY_EDGE,
	EXTI_TRIGGER_LAST
} EXTI_trigger_t;

/*** EXTI functions ***/

void EXTI_init(void);
void EXTI_configure_gpio(const GPIO_pin_t* gpio, EXTI_trigger_t trigger);
void EXTI_configure_line(EXTI_line_t line, EXTI_trigger_t trigger);
void EXTI_clear_flag(EXTI_line_t line);
void EXTI_clear_all_flags(void);

#endif /* __EXTI_H__ */
