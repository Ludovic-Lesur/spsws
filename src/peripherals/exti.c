/*
 * exti.c
 *
 *  Created on: 18 juin 2018
 *      Author: Ludo
 */

#include "exti.h"

#include "exti_reg.h"
#include "gpio.h"
#include "mapping.h"
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

	/* Configure DIO2 and DIO3 as input */
	GPIO_Configure(GPIO_DIO2, Input, OpenDrain, HighSpeed, NoPullUpNoPullDown);
#ifndef USE_SX1232_DIOX
	GPIO_Configure(GPIO_DIO3, Input, OpenDrain, HighSpeed, NoPullUpNoPullDown);
#endif

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
