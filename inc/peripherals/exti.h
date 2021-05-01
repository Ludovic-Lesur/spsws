/*
 * exti.h
 *
 *  Created on: 18 june 2018
 *      Author: Ludo
 */

#ifndef EXTI_H
#define EXTI_H

#include "gpio.h"

/*** EXTI structures ***/

typedef enum {
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
	EXTI_LINE_LPTIM1 = 29
} EXTI_Line;

typedef enum {
	EXTI_TRIGGER_RISING_EDGE,
	EXTI_TRIGGER_FALLING_EDGE,
	EXTI_TRIGGER_ANY_EDGE
} EXTI_Trigger;

/*** EXTI functions ***/

void EXTI_Init(void);
void EXTI_ConfigureGpio(const GPIO* gpio, EXTI_Trigger edge_trigger);
void EXTI_ConfigureLine(EXTI_Line line, EXTI_Trigger edge_trigger);
void EXTI_ClearAllFlags(void);

#endif /* EXTI_H */
