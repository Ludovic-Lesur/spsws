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
void __attribute__((optimize("-O0"))) EXTI0_1_IRQHandler(void) {
	// Unused.
}

/* EXTI LINES 2-3 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) EXTI2_3_IRQHandler(void) {
	// Unused.
}

/* EXTI LINES 4-15 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void __attribute__((optimize("-O0"))) EXTI4_15_IRQHandler(void) {
#if (defined CM || defined ATM)
	// Speed edge interrupt.
	if (((EXTI -> PR) & (0b1 << (GPIO_WIND_SPEED.pin_index))) != 0) {
		// Manage callback.
		if (((EXTI -> IMR) & (0b1 << (GPIO_WIND_SPEED.pin_index))) != 0) {
			WIND_speed_edge_callback();
		}
		// Clear flag.
		EXTI -> PR |= (0b1 << (GPIO_WIND_SPEED.pin_index)); // PIFx='1' (writing '1' clears the bit).
	}
#ifdef WIND_VANE_ULTIMETER
	// Direction edge interrupt.
	if (((EXTI -> PR) & (0b1 << (GPIO_WIND_DIRECTION.pin_index))) != 0) {
		// Manage callback.
		if (((EXTI -> IMR) & (0b1 << (GPIO_WIND_DIRECTION.pin_index))) != 0) {
			WIND_direction_edge_callback();
		}
		// Clear flag.
		EXTI -> PR |= (0b1 << (GPIO_WIND_DIRECTION.pin_index)); // PIFx='1' (writing '1' clears the bit).
	}
#endif
	// Rain edge interrupt.
	if (((EXTI -> PR) & (0b1 << (GPIO_RAIN.pin_index))) != 0) {
		// Manage callback.
		if (((EXTI -> IMR) & (0b1 << (GPIO_RAIN.pin_index))) != 0) {
			RAIN_edge_callback();
		}
		// Clear flag.
		EXTI -> PR |= (0b1 << (GPIO_RAIN.pin_index)); // PIFx='1' (writing '1' clears the bit).
	}
#endif
}

/* SET EXTI TRIGGER.
 * @param bit_idx:	Interrupt index.
 * @return status:	Function execution status.
 */
static void EXTI_set_trigger(EXTI_trigger_t trigger, unsigned char bit_idx) {
	// Check index.
	if (bit_idx > EXTI_RTSR_FTSR_MAX_INDEX) return;
	// Select triggers.
	switch (trigger) {
	// Rising edge only.
	case EXTI_TRIGGER_RISING_EDGE:
		EXTI -> RTSR |= (0b1 << bit_idx); // Rising edge enabled.
		EXTI -> FTSR &= ~(0b1 << bit_idx); // Falling edge disabled.
		break;
	// Falling edge only.
	case EXTI_TRIGGER_FALLING_EDGE:
		EXTI -> RTSR &= ~(0b1 << bit_idx); // Rising edge disabled.
		EXTI -> FTSR |= (0b1 << bit_idx); // Falling edge enabled.
		break;
	// Both edges.
	case EXTI_TRIGGER_ANY_EDGE:
		EXTI -> RTSR |= (0b1 << bit_idx); // Rising edge enabled.
		EXTI -> FTSR |= (0b1 << bit_idx); // Falling edge enabled.
		break;
	// Unknown configuration.
	default:
		break;
	}
	// Clear flag.
	EXTI -> PR |= (0b1 << bit_idx);
}

/*** EXTI functions ***/

/* INIT EXTI PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void EXTI_init(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 0); // SYSCFEN='1'.
	// Mask all sources by default.
	EXTI -> IMR = 0;
	// Clear all flags.
	EXTI -> PR |= 0x007BFFFF; // PIFx='1'.
	// Set interrupts priority.
	NVIC_set_priority(NVIC_IT_EXTI_0_1, 3);
	NVIC_set_priority(NVIC_IT_EXTI_4_15, 0);
}

/* CONFIGURE A GPIO AS EXTERNAL INTERRUPT SOURCE.
 * @param gpio:		GPIO to be attached to EXTI peripheral.
 * @edge_trigger:	Interrupt edge trigger (see EXTI_trigger_t epin_indexeration in exti.h).
 * @return:			None.
 */
void EXTI_configure_gpio(const GPIO_pin_t* gpio, EXTI_trigger_t trigger) {
	// Select GPIO port.
	SYSCFG -> EXTICR[((gpio -> pin_index) / 4)] &= ~(0b1111 << (4 * ((gpio -> pin_index) % 4)));
	SYSCFG -> EXTICR[((gpio -> pin_index) / 4)] |= ((gpio -> port_index) << (4 * ((gpio -> pin_index) % 4)));
	// Set mask and trigger.
	EXTI -> IMR |= (0b1 << ((gpio -> pin_index))); // IMx='1'.
	// Select triggers.
	EXTI_set_trigger(trigger, (gpio -> pin_index));
}

/* CONFIGURE A LINE AS INTERNAL INTERRUPT SOURCE.
 * @param line:		Line to configure (see EXTI_line_t enum).
 * @edge_trigger:	Interrupt edge trigger (see EXTI_trigger_t enum).
 * @return:			None.
 */
void EXTI_configure_line(EXTI_line_t line, EXTI_trigger_t trigger) {
	// Check line.
	if (line >= EXTI_LINE_LAST) return;
	// Set mask.
	EXTI -> IMR |= (0b1 << line); // IMx='1'.
	// Select triggers.
	if (line <= EXTI_RTSR_FTSR_MAX_INDEX) {
		EXTI_set_trigger(trigger, line);
	}
}

/* CLEAR ALL EXTI FLAGS.
 * @param:	None.
 * @return:	None.
 */
void EXTI_clear_all_flags(void) {
	// Clear all flags.
	EXTI -> PR |= 0x007BFFFF; // PIFx='1'.
}
