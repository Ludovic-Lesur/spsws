/*
 * exti.c
 *
 *  Created on: 18 juin 2018
 *      Author: Ludovic
 */

#include "exti.h"

#include "exti_reg.h"
#include "gpio_reg.h"
#include "nvic.h"
#include "rcc_reg.h"
#include "syscfg_reg.h"
#include "tim.h"
#include "ultimeter.h"
#include "usart.h"

/*** EXTI local macros ***/

#define EXTI_FILTER_DURATION_MS	10

/*** EXTI local structures ***/

typedef struct {
	unsigned int exti15_period; // TIM2 counter value between 2 edge interrupts on wind speed input.
	unsigned int exti8_phase_shift; // TIM2 counter value when edge interrupt detected on wind direction input.
} EXTI_Context;

/*** EXTI local global variables ***/

volatile EXTI_Context exti_ctx;

/*** EXTI functions ***/

/* CONFIGURE EXTERNAL INTERRUPTS TRIGGERED BY GPIO.
 * @param:	None.
 * @return:	None.
 */
void EXTI_Init(void) {

	/* Init context */
	exti_ctx.exti15_period = 0;
	exti_ctx.exti8_phase_shift = 0;

	/* Enable peripheral clock */
	RCC -> APB2ENR |= (0b1 << 0); // SYSCFEN='1'.

	/* Configure PA15 (DIO2) and PA8 (DIO3) as input */
	RCC -> IOPENR |= (0b1 << 0); // GPIOAEN='1'.
	GPIOA -> MODER &= ~(0b11 << 30); // MODE15='00'.
	GPIOA -> MODER &= ~(0b11 << 16); // MODE8='00'.

	/* Attach rising edge external interrupt on PA15 (DIO2) and PA8 (DIO3) pins */
	EXTI -> IMR |= (0b1 << 15) | (0b1 << 8); // IM8='1' and IM15='1', all other masked.
	EXTI -> RTSR |= (0b1 << 15) | (0b1 << 8); // Rising edge trigger on PA8 and PA15.
	EXTI -> FTSR = 0; // None falling edge trigger.
	EXTI -> SWIER = 0; // None software interrupt.
	SYSCFG -> EXTICR3 &= 0xFFFFFFF0; // Select port A: EXTI8='0000'.
	SYSCFG -> EXTICR4 &= 0xFFFF0FFF; // Select port A: EXTI15='0000'.

	/* Disable interrupt by default */
	NVIC_DisableInterrupt(IT_EXTI4_15);
}

/* EXTI LINES 4-15 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void EXTI4_15_IRQHandler(void) {

	/* PA15 (DIO2) edge interrupt */
	if (((EXTI -> PR) & (0b1 << 15)) != 0) {
		// Clear flag.
		EXTI -> PR |= (0b1 << 15); // PIF15='1' (writing '1' clears the bit).
		// Rising edge on speed signal.
		ULTIMETER_IncrementWindSpeedCount();
		// Capture period.
		TIM2_Stop();
		exti_ctx.exti15_period = TIM2_GetCounter();
		// Compute direction
		if ((exti_ctx.exti15_period > 0) && (exti_ctx.exti8_phase_shift <= exti_ctx.exti15_period)) {
			unsigned char direction_pourcent = (exti_ctx.exti8_phase_shift * 100) / (exti_ctx.exti15_period);
			ULTIMETER_UpdateWindDirection(direction_pourcent);
		}
		// Start new cycle.
		TIM2_Restart();
	}


	/* PA8 (DIO3) edge interrupt */
	if (((EXTI -> PR) & (0b1 << 8)) != 0) {
		// Clear flag.
		EXTI -> PR |= (0b1 << 8); // PIF8='1' (writing '1' clears the bit).
		// Rising edge on direction signal.
		exti_ctx.exti8_phase_shift = TIM2_GetCounter();
	}
}
