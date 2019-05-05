/*
 * exti.c
 *
 *  Created on: 18 juin 2018
 *      Author: Ludo
 */

#include "exti.h"

#include "exti_reg.h"
#include "mapping.h"
#include "nvic.h"
#include "rcc_reg.h"
#include "rain.h"
#include "syscfg_reg.h"
#include "wind.h"

/*** EXTI local functions ***/

/* EXTI LINES 0-1 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void EXTI0_1_IRQHandler(void) {
	// Unused.
}

/* EXTI LINES 2-3 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void EXTI2_3_IRQHandler(void) {
	// Unused.
}

/* EXTI LINES 4-15 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void EXTI4_15_IRQHandler(void) {

#if (defined CM_RTC || defined ATM)
	/* Speed edge interrupt */
	if (((EXTI -> PR) & (0b1 << (GPIO_WIND_SPEED.num))) != 0) {
		// Clear flag.
		EXTI -> PR |= (0b1 << (GPIO_WIND_SPEED.num)); // PIFx='1' (writing '1' clears the bit).
		// Call WIND callback.
		WIND_SpeedEdgeCallback();
	}

#ifdef WIND_VANE_ULTIMETER
	/* Direction edge interrupt */
	if (((EXTI -> PR) & (0b1 << (GPIO_WIND_DIRECTION.num))) != 0) {
		// Clear flag.
		EXTI -> PR |= (0b1 << (GPIO_WIND_DIRECTION.num)); // PIFx='1' (writing '1' clears the bit).
		// Call WIND callback.
		WIND_DirectionEdgeCallback();
	}
#endif

	/* Rain edge interrupt */
	if (((EXTI -> PR) & (0b1 << (GPIO_RAIN.num))) != 0) {
		// Clear flag.
		EXTI -> PR |= (0b1 << (GPIO_RAIN.num)); // PIFx='1' (writing '1' clears the bit).
		// Call WIND callback.
		RAIN_EdgeCallback();
	}
#endif
}

/*** EXTI functions ***/

/* INIT EXTI PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void EXTI_Init(void) {

	/* Enable peripheral clock */
	RCC -> APB2ENR |= (0b1 << 0); // SYSCFEN='1'.

	/* Disable interrupts by default */
	NVIC_DisableInterrupt(IT_EXTI_0_1);
	NVIC_DisableInterrupt(IT_EXTI_2_3);
	NVIC_DisableInterrupt(IT_EXTI_4_15);
}

/* CONFIGURE A GPIO AS EXTERNAL INTERRUPT SOURCE.
 * @param gpio:		GPIO to be attached to EXTI peripheral.
 * @edge_trigger:	Interrupt edge trigger (see EXTI_Trigger enumeration in exti.h).
 * @return:			None.
 */
void EXTI_ConfigureInterrupt(GPIO* gpio, EXTI_Trigger edge_trigger) {

	/* Select GPIO port */
	SYSCFG -> EXTICR[((gpio -> num) / 4)] &= ~(0b1111 << (4 * ((gpio -> num) % 4)));
	SYSCFG -> EXTICR[((gpio -> num) / 4)] |= ((gpio -> port_index) << (4 * ((gpio -> num) % 4)));

	/* Select triggers */
	switch (edge_trigger) {
	// Rising edge only.
	case EXTI_TRIGGER_RISING_EDGE:
		EXTI -> IMR |= (0b1 << ((gpio -> num))); // IMx='1'.
		EXTI -> RTSR |= (0b1 << ((gpio -> num))); // Rising edge enabled.
		EXTI -> FTSR &= ~(0b1 << ((gpio -> num))); // Falling edge disabled.
		break;
	// Falling edge only.
	case EXTI_TRIGGER_FALLING_EDGE:
		EXTI -> IMR |= (0b1 << ((gpio -> num))); // IMx='1'.
		EXTI -> RTSR &= ~(0b1 << ((gpio -> num))); // Rising edge disabled.
		EXTI -> FTSR |= (0b1 << ((gpio -> num))); // Falling edge enabled.
		break;
	// Both edges.
	case EXTI_TRIGGER_ANY_EDGE:
		EXTI -> IMR |= (0b1 << ((gpio -> num))); // IMx='1'.
		EXTI -> RTSR |= (0b1 << ((gpio -> num))); // Rising edge enabled.
		EXTI -> FTSR |= (0b1 << ((gpio -> num))); // Falling edge enabled.
		break;
	// Unknown configuration.
	default:
		EXTI -> IMR &= ~(0b1 << ((gpio -> num))); // IMx='0'.
		EXTI -> RTSR &= ~(0b1 << ((gpio -> num))); // Rising edge disabled.
		EXTI -> FTSR &= ~(0b1 << ((gpio -> num))); // Falling edge disabled.
		break;
	}
}
