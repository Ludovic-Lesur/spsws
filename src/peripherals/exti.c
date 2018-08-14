/*
 * exti.c
 *
 *  Created on: 18 juin 2018
 *      Author: Ludovic
 */

#ifdef CONTINUOUS_MODE

#include "exti.h"

#include "exti_reg.h"
#include "gpio_reg.h"
#include "nvic.h"
#include "rcc_reg.h"
#include "syscfg_reg.h"

/*** EXTI functions ***/

/* CONFIGURE EXTERNAL INTERRUPTS TRIGGERED BY GPIO.
 * @param:	None.
 * @return:	None.
 */
void EXTI_Init(void) {

	/* Enable peripheral clock */
	RCC -> APB2ENR |= (0b1 << 0); // SYSCFEN='1'.

	/* Configure PA11 as input */
	RCC -> IOPENR |= (0b1 << 0); // GPIOAEN='1'.
	GPIOA -> MODER &= ~(0b11 << 22); // MODE11='00'.

	/* Attach rising edge external interrupt on PA11 pin */
	EXTI -> IMR = (0b1 << 11); // IM11='1', all other masked.
	EXTI -> EMR = 0; // All events masked.
	EXTI -> RTSR = (0b1 << 11); // RT11='1', all other resetted.
	EXTI -> FTSR = 0; // None falling edge trigger.
	EXTI -> SWIER = 0; // None software interrupt.
	SYSCFG -> EXTICR3 &= 0xFFFF0FFF; // EXTI11='0000'.

	/* Enable interrupt */
	NVIC_EnableInterrupt(IT_EXTI4_15);
}

/* EXTI LINES 4-15 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void EXTI4_15_IRQHandler(void) {

	/* PA11 external interrupt */

	if (((EXTI -> PR) & (0b1 << 11)) != 0) {
		// Clear flag.
		EXTI -> PR |= (0b1 << 11); // PIF11='1' (writing '1' clears the bit).
	}
}

#endif
