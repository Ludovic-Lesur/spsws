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

/*** EXTI local macros ***/

#define EXTI_RTSR_FTSR_MAX_INDEX	22

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
#if (defined CM || defined ATM)
	// Speed edge interrupt.
	if (((EXTI -> PR) & (0b1 << (GPIO_WIND_SPEED.gpio_num))) != 0) {
		// Clear flag.
		EXTI -> PR |= (0b1 << (GPIO_WIND_SPEED.gpio_num)); // PIFx='1' (writing '1' clears the bit).
		// Call WIND callback.
		WIND_SpeedEdgeCallback();
	}
#ifdef WIND_VANE_ULTIMETER
	// Direction edge interrupt.
	if (((EXTI -> PR) & (0b1 << (GPIO_WIND_DIRECTION.gpio_num))) != 0) {
		// Clear flag.
		EXTI -> PR |= (0b1 << (GPIO_WIND_DIRECTION.gpio_num)); // PIFx='1' (writing '1' clears the bit).
		// Call WIND callback.
		WIND_DirectionEdgeCallback();
	}
#endif
	// Rain edge interrupt.
	if (((EXTI -> PR) & (0b1 << (GPIO_RAIN.gpio_num))) != 0) {
		// Clear flag.
		EXTI -> PR |= (0b1 << (GPIO_RAIN.gpio_num)); // PIFx='1' (writing '1' clears the bit).
		// Call RAIN callback.
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
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 0); // SYSCFEN='1'.
	// Mask all sources by default.
	EXTI -> IMR = 0;
	// Clear all flags.
	EXTI -> PR |= 0x007BFFFF; // PIFx='1'.
	// Set interrupts priority.
	NVIC_SetPriority(NVIC_IT_EXTI_0_1, 3);
	NVIC_SetPriority(NVIC_IT_EXTI_4_15, 0);
}

/* CONFIGURE A GPIO AS EXTERNAL INTERRUPT SOURCE.
 * @param gpio:		GPIO to be attached to EXTI peripheral.
 * @edge_trigger:	Interrupt edge trigger (see EXTI_Trigger egpio_numeration in exti.h).
 * @return:			None.
 */
void EXTI_ConfigureGpio(const GPIO* gpio, EXTI_Trigger edge_trigger) {
	// Select GPIO port.
	SYSCFG -> EXTICR[((gpio -> gpio_num) / 4)] &= ~(0b1111 << (4 * ((gpio -> gpio_num) % 4)));
	SYSCFG -> EXTICR[((gpio -> gpio_num) / 4)] |= ((gpio -> gpio_port_index) << (4 * ((gpio -> gpio_num) % 4)));
	// Select triggers.
	switch (edge_trigger) {
	// Rising edge only.
	case EXTI_TRIGGER_RISING_EDGE:
		EXTI -> IMR |= (0b1 << ((gpio -> gpio_num))); // IMx='1'.
		EXTI -> RTSR |= (0b1 << ((gpio -> gpio_num))); // Rising edge enabled.
		EXTI -> FTSR &= ~(0b1 << ((gpio -> gpio_num))); // Falling edge disabled.
		break;
	// Falling edge only.
	case EXTI_TRIGGER_FALLING_EDGE:
		EXTI -> IMR |= (0b1 << ((gpio -> gpio_num))); // IMx='1'.
		EXTI -> RTSR &= ~(0b1 << ((gpio -> gpio_num))); // Rising edge disabled.
		EXTI -> FTSR |= (0b1 << ((gpio -> gpio_num))); // Falling edge enabled.
		break;
	// Both edges.
	case EXTI_TRIGGER_ANY_EDGE:
		EXTI -> IMR |= (0b1 << ((gpio -> gpio_num))); // IMx='1'.
		EXTI -> RTSR |= (0b1 << ((gpio -> gpio_num))); // Rising edge enabled.
		EXTI -> FTSR |= (0b1 << ((gpio -> gpio_num))); // Falling edge enabled.
		break;
	// Unknown configuration.
	default:
		EXTI -> IMR &= ~(0b1 << ((gpio -> gpio_num))); // IMx='0'.
		EXTI -> RTSR &= ~(0b1 << ((gpio -> gpio_num))); // Rising edge disabled.
		EXTI -> FTSR &= ~(0b1 << ((gpio -> gpio_num))); // Falling edge disabled.
		break;
	}
	// Clear flag.
	EXTI -> PR |= (0b1 << ((gpio -> gpio_num)));
}

/* CONFIGURE A LINE AS INTERNAL INTERRUPT SOURCE.
 * @param line:		Line to configure (see EXTI_Line enum).
 * @edge_trigger:	Interrupt edge trigger (see EXTI_Trigger enum).
 * @return:			None.
 */
void EXTI_ConfigureLine(EXTI_Line line, EXTI_Trigger edge_trigger) {
	// Select triggers.
	switch (edge_trigger) {
	// Rising edge only.
	case EXTI_TRIGGER_RISING_EDGE:
		EXTI -> IMR |= (0b1 << line); // IMx='1'.
		if (line <= EXTI_RTSR_FTSR_MAX_INDEX) {
			EXTI -> RTSR |= (0b1 << line); // Rising edge enabled.
			EXTI -> FTSR &= ~(0b1 << line); // Falling edge disabled.
		}
		break;
	// Falling edge only.
	case EXTI_TRIGGER_FALLING_EDGE:
		EXTI -> IMR |= (0b1 << line); // IMx='1'.
		if (line <= EXTI_RTSR_FTSR_MAX_INDEX) {
			EXTI -> RTSR &= ~(0b1 << line); // Rising edge disabled.
			EXTI -> FTSR |= (0b1 << line); // Falling edge enabled.
		}
		break;
	// Both edges.
	case EXTI_TRIGGER_ANY_EDGE:
		EXTI -> IMR |= (0b1 << line); // IMx='1'.
		if (line <= EXTI_RTSR_FTSR_MAX_INDEX) {
			EXTI -> RTSR |= (0b1 << line); // Rising edge enabled.
			EXTI -> FTSR |= (0b1 << line); // Falling edge enabled.
		}
		break;
	// Unknown configuration.
	default:
		break;
	}
	// Clear flag.
	if (line <= EXTI_RTSR_FTSR_MAX_INDEX) {
		EXTI -> PR |= (0b1 << line);
	}
}

/* CLEAR ALL EXTI FLAGS.
 * @param:	None.
 * @return:	None.
 */
void EXTI_ClearAllFlags(void) {
	// Clear all flags.
	EXTI -> PR |= 0x007BFFFF; // PIFx='1'.
}
