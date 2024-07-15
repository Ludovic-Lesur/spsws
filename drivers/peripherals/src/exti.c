/*
 * exti.c
 *
 *  Created on: 18 jun. 2018
 *      Author: Ludo
 */

#include "exti.h"

#include "exti_reg.h"
#include "gpio.h"
#include "mapping.h"
#include "rcc_reg.h"
#include "syscfg_reg.h"
#include "types.h"

/*** EXTI local macros ***/

#define EXTI_RTSR_FTSR_RESERVED_INDEX	18
#define EXTI_RTSR_FTSR_MAX_INDEX		22

/*** EXTI local global variables ***/

static EXTI_gpio_irq_cb_t exti_gpio_irq_callbacks[GPIO_PINS_PER_PORT];

/*** EXTI local functions ***/

/*******************************************************************/
#define _EXTI_irq_handler(gpio) { \
	/* Check flag */ \
	if (((EXTI -> PR) & (0b1 << (gpio.pin))) != 0) { \
		/* Check mask and callback */ \
		if ((((EXTI -> IMR) & (0b1 << (gpio.pin))) != 0) && (exti_gpio_irq_callbacks[gpio.pin] != NULL)) { \
			/* Execute callback */ \
			exti_gpio_irq_callbacks[gpio.pin](); \
		} \
		/* Clear flag */ \
		EXTI -> PR |= (0b1 << (gpio.pin)); \
	} \
}

#ifdef HW2_0
/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI0_1_IRQHandler(void) {
	// SX1232 DIO0 (PB1).
	_EXTI_irq_handler(GPIO_SX1232_DIO0);
}
#endif

#ifdef HW1_0
/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI2_3_IRQHandler(void) {
	// SX1232 DIO0 (PB2).
	_EXTI_irq_handler(GPIO_SX1232_DIO0);
}
#endif

#ifdef HW2_0
/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI4_15_IRQHandler(void) {
	// DIO0 (PA10).
	_EXTI_irq_handler(GPIO_DIO0);
	// DIO1 (PA11).
	_EXTI_irq_handler(GPIO_DIO1);
	// DIO2 (PA15).
	_EXTI_irq_handler(GPIO_DIO2);
}
#endif

/*******************************************************************/
static void _EXTI_set_trigger(EXTI_trigger_t trigger, uint8_t line_idx) {
	// Select triggers.
	switch (trigger) {
	// Rising edge only.
	case EXTI_TRIGGER_RISING_EDGE:
		EXTI -> RTSR |= (0b1 << line_idx); // Rising edge enabled.
		EXTI -> FTSR &= ~(0b1 << line_idx); // Falling edge disabled.
		break;
	// Falling edge only.
	case EXTI_TRIGGER_FALLING_EDGE:
		EXTI -> RTSR &= ~(0b1 << line_idx); // Rising edge disabled.
		EXTI -> FTSR |= (0b1 << line_idx); // Falling edge enabled.
		break;
	// Both edges.
	case EXTI_TRIGGER_ANY_EDGE:
		EXTI -> RTSR |= (0b1 << line_idx); // Rising edge enabled.
		EXTI -> FTSR |= (0b1 << line_idx); // Falling edge enabled.
		break;
	// Unknown configuration.
	default:
		break;
	}
	// Clear flag.
	EXTI -> PR |= (0b1 << line_idx);
}

/*** EXTI functions ***/

/*******************************************************************/
void EXTI_init(void) {
	// Local variables.
	uint8_t idx = 0;
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 0); // SYSCFEN='1'.
	// Mask all sources by default.
	EXTI -> IMR = 0;
	// Clear all flags.
	EXTI -> PR |= 0x007BFFFF; // PIFx='1'.
	// Reset callbacks.
	for (idx=0 ; idx<GPIO_PINS_PER_PORT ; idx++) {
		exti_gpio_irq_callbacks[idx] = NULL;
	}
}

/*******************************************************************/
void EXTI_configure_gpio(const GPIO_pin_t* gpio, EXTI_trigger_t trigger, EXTI_gpio_irq_cb_t irq_callback) {
	// Select GPIO port.
	SYSCFG -> EXTICR[((gpio -> pin) >> 2)] &= ~(0b1111 << (((gpio -> pin) % 4) << 2));
	SYSCFG -> EXTICR[((gpio -> pin) >> 2)] |= ((gpio -> port_index) << (((gpio -> pin) % 4) << 2));
	// Set mask.
	EXTI -> IMR |= (0b1 << ((gpio -> pin))); // IMx='1'.
	// Select triggers.
	_EXTI_set_trigger(trigger, (gpio -> pin));
	// Register callback.
	exti_gpio_irq_callbacks[gpio -> pin] = irq_callback;
}

/*******************************************************************/
void EXTI_release_gpio(const GPIO_pin_t* gpio) {
	// Set mask.
	EXTI -> IMR &= ~(0b1 << ((gpio -> pin))); // IMx='0'.
}

/*******************************************************************/
void EXTI_configure_line(EXTI_line_t line, EXTI_trigger_t trigger) {
	// Set mask.
	EXTI -> IMR |= (0b1 << line); // IMx='1'.
	// Select triggers.
	if ((line != EXTI_RTSR_FTSR_RESERVED_INDEX) || (line <= EXTI_RTSR_FTSR_MAX_INDEX)) {
		_EXTI_set_trigger(trigger, line);
	}
}

/*******************************************************************/
void EXTI_clear_flag(EXTI_line_t line) {
	// Clear flag.
	EXTI -> PR |= line; // PIFx='1'.
}
