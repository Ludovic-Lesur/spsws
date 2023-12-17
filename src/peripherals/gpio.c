/*
 * gpio.c
 *
 *  Created on: 16 dec. 2018
 *      Author: Ludo
 */

#include "gpio.h"

#include "gpio_reg.h"
#include "mapping.h"
#include "rcc_reg.h"
#include "types.h"

/*** GPIO local macros ***/

#define GPIO_AFRH_OFFSET 	8 	// Limit between AFRL and AFRH registers.

/*** GPIO local functions ***/

/*******************************************************************/
static void _GPIO_set_mode(const GPIO_pin_t* gpio, GPIO_mode_t mode) {
	// Local variables.
	uint32_t reg_value = 0;
	// Read register.
	reg_value = ((gpio -> port) -> MODER);
	// Analog mode by default.
	reg_value |= (0b11 << ((gpio -> pin) << 1)); // MODERy = '11'.
	// Set required bits.
	switch(mode) {
	case GPIO_MODE_INPUT:
		// MODERy = '00'.
		reg_value &= ~(0b11 << ((gpio -> pin) << 1));
		break;
	case GPIO_MODE_OUTPUT:
		// MODERy = '01'.
		reg_value &= ~(0b1 << (((gpio -> pin) << 1) + 1));
		break;
	case GPIO_MODE_ALTERNATE_FUNCTION:
		// MODERy = '10'.
		reg_value &= ~(0b1 << ((gpio -> pin) << 1));
		break;
	case GPIO_MODE_ANALOG:
		// Nothing to do.
		break;
	default:
		break;
	}
	// Write register.
	(gpio -> port) -> MODER = reg_value;
}

/*******************************************************************/
static GPIO_mode_t _GPIO_get_mode(const GPIO_pin_t* gpio) {
	// Get mode.
	GPIO_mode_t gpio_mode = (((gpio -> port) -> MODER) & (0b11 << ((gpio -> pin) << 1))) >> ((gpio -> pin) << 1);
	return gpio_mode;
}

/*******************************************************************/
static void _GPIO_set_output_type(const GPIO_pin_t* gpio, GPIO_output_type_t output_type) {
	// Set bit.
	switch(output_type) {
	case GPIO_TYPE_PUSH_PULL:
		// OTy = '0'.
		(gpio -> port) -> OTYPER &= ~(0b1 << (gpio -> pin));
		break;
	case GPIO_TYPE_OPEN_DRAIN:
		// OTy = '1'.
		(gpio -> port) -> OTYPER |= (0b1 << (gpio -> pin));
		break;
	default:
		break;
	}
}

/*******************************************************************/
static void _GPIO_set_output_speed(const GPIO_pin_t* gpio, GPIO_output_speed_t output_speed) {
	// Local variables.
	uint32_t reg_value = 0;
	// Read register.
	reg_value = ((gpio -> port) -> OSPEEDR);
	// Low speed by default.
	reg_value &= ~(0b11 << ((gpio -> pin) << 1));
	// Set required bits.
	switch(output_speed) {
	case GPIO_SPEED_LOW:
		// Nothing to do.
		break;
	case GPIO_SPEED_MEDIUM:
		// OSPEEDRy = '01'.
		reg_value |= (0b1 << ((gpio -> pin) << 1));
		break;
	case GPIO_SPEED_HIGH:
		// OSPEEDRy = '10'.
		reg_value |= (0b1 << (((gpio -> pin) << 1) + 1));
		break;
	case GPIO_SPEED_VERY_HIGH:
		// OSPEEDRy = '11'.
		reg_value |= (0b11 << ((gpio -> pin) << 1));
		break;
	default:
		break;
	}
	// Write register.
	(gpio -> port) -> OSPEEDR = reg_value;
}

/*******************************************************************/
static void _GPIO_set_pull_resistor(const GPIO_pin_t* gpio, GPIO_pull_resistor_t pull_resistor) {
	// Local variables.
	uint32_t reg_value = 0;
	// Read registers.
	reg_value = ((gpio -> port) -> PUPDR);
	// Disable resistors by default.
	reg_value &= ~(0b11 << ((gpio -> pin) << 1));
	// Set required bits.
	switch(pull_resistor) {
	case GPIO_PULL_NONE:
		// Nothing to do.
		break;
	case GPIO_PULL_UP:
		// PUPDRy = '01'.
		reg_value |= (0b1 << ((gpio -> pin) << 1));
		break;
	case GPIO_PULL_DOWN:
		// PUPDRy = '10'.
		reg_value |= (0b1 << (((gpio -> pin) << 1) + 1));
		break;
	default:
		break;
	}
	// Write register.
	(gpio -> port) -> PUPDR = reg_value;
}

/*******************************************************************/
static void _GPIO_set_alternate_function(const GPIO_pin_t* gpio, uint32_t alternate_function) {
	// Local variables.
	uint32_t reg_value = 0;
	// Clamp AF number.
	alternate_function &= 0x0F;
	// Select proper register to set.
	if ((gpio -> pin) < GPIO_AFRH_OFFSET) {
		// Read register.
		reg_value = ((gpio -> port) -> AFRL);
		// Set AFRL register: AFRy = 'alternate_function'.
		reg_value &= ~(0b1111 << ((gpio -> pin) << 2));
		reg_value |= (alternate_function << ((gpio -> pin) << 2));
		// Write register.
		(gpio -> port) -> AFRL = reg_value;
	}
	else {
		// Read register.
		reg_value = ((gpio -> port) -> AFRH);
		// Set AFRH register: AFRy = 'alternate_function'.
		reg_value &= ~(0b1111 << (((gpio -> pin) - GPIO_AFRH_OFFSET) << 2));
		reg_value |= (alternate_function << (((gpio -> pin) - GPIO_AFRH_OFFSET) << 2));
		// Write register.
		(gpio -> port) -> AFRH = reg_value;
	}
}

/*** GPIO functions ***/

/*******************************************************************/
void GPIO_configure(const GPIO_pin_t* gpio, GPIO_mode_t mode, GPIO_output_type_t output_type, GPIO_output_speed_t output_speed, GPIO_pull_resistor_t pull_resistor) {
	// Configure GPIO.
	_GPIO_set_output_type(gpio, output_type);
	_GPIO_set_output_speed(gpio, output_speed);
	_GPIO_set_pull_resistor(gpio, pull_resistor);
	_GPIO_set_alternate_function(gpio, (gpio -> alternate_function));
	_GPIO_set_mode(gpio, mode);
}

/*******************************************************************/
void GPIO_init(void) {
	// Enable all GPIOx clocks.
	RCC -> IOPENR |= (0b111 << 0); // IOPxEN='1'.
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) GPIO_write(const GPIO_pin_t* gpio, uint8_t state) {
	// Set bit.
	if (state == 0) {
		(gpio -> port) -> ODR &= ~(0b1 << (gpio -> pin));
	}
	else {
		(gpio -> port) -> ODR |= (0b1 << (gpio -> pin));
	}
}

/*******************************************************************/
uint8_t __attribute__((optimize("-O0"))) GPIO_read(const GPIO_pin_t* gpio) {
	// Check mode.
	uint8_t state = 0;
	switch (_GPIO_get_mode(gpio)) {
	case GPIO_MODE_INPUT:
		// GPIO configured as input -> read IDR register.
		if ((((gpio -> port) -> IDR) & (0b1 << (gpio -> pin))) != 0) {
			state = 1;
		}
		break;
	case GPIO_MODE_OUTPUT:
		// GPIO configured as output -> read ODR register.
		if ((((gpio -> port) -> ODR) & (0b1 << (gpio -> pin))) != 0) {
			state = 1;
		}
		break;
	default:
		break;
	}
	return state;
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) GPIO_toggle(const GPIO_pin_t* gpio) {
	// Toggle ODR bit.
	(gpio -> port) -> ODR ^= (0b1 << (gpio -> pin));
}
