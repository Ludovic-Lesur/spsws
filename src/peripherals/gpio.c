/*
 * gpio.c
 *
 *  Created on: 16 dec. 2018
 *      Author: Ludo
 */

#include "gpio.h"

#include "gpio_reg.h"
#include "mapping.h"
#include "mode.h"
#include "rcc_reg.h"

/*** GPIO local macros ***/

#define GPIO_AFRH_OFFSET 	8 	// Limit between AFRL and AFRH registers.

/*** GPIO local functions ***/

/* SET THE MODE OF A GPIO PIN.
 * @param gpio:	GPIO structure.
 * @param mode: Mode (see enum defined in gpio.h).
 * @return: 	None.
 */
static void GPIO_set_mode(const GPIO_pin_t* gpio, GPIO_mode_t mode) {
	// Set analog mode during transition.
	(gpio -> gpio_port_address) -> MODER |= (0b11 << (2 * (gpio -> gpio_num))); // MODERy = '11'.
	// Set required bits.
	switch(mode) {
	case GPIO_MODE_INPUT:
		// MODERy = '00'.
		(gpio -> gpio_port_address) -> MODER &= ~(0b11 << (2 * (gpio -> gpio_num)));
		break;
	case GPIO_MODE_OUTPUT:
		// MODERy = '01'.
		(gpio -> gpio_port_address) -> MODER &= ~(0b1 << (2 * (gpio -> gpio_num) + 1));
		break;
	case GPIO_MODE_ALTERNATE_FUNCTION:
		// MODERy = '10'.
		(gpio -> gpio_port_address) -> MODER &= ~(0b1 << (2 * (gpio -> gpio_num)));
		break;
	case GPIO_MODE_ANALOG:
		// Nothing to do.
		break;
	default:
		break;
	}
}

/* GET THE MODE OF A GPIO PIN.
 * @param gpio:			GPIO structure.
 * @return gpioMode: 	Current mode (see enum defined in gpio.h).
 */
static GPIO_mode_t GPIO_get_mode(const GPIO_pin_t* gpio) {
	// Get mode.
	GPIO_mode_t gpio_mode = (((gpio -> gpio_port_address) -> MODER) & (0b11 << (2 * (gpio -> gpio_num)))) >> (2 * (gpio -> gpio_num));
	return gpio_mode;
}

/* SET THE OUTPUT TYPE OF A GPIO PIN.
 * @param gpio:			GPIO structure.
 * @param output_type: 	Output type (see enum defined in gpio.h).
 * @return: 			None.
 */
static void GPIO_set_output_type(const GPIO_pin_t* gpio, GPIO_output_type_t output_type) {
	// Set bit.
	switch(output_type) {
	case GPIO_TYPE_PUSH_PULL:
		// OTy = '0'.
		(gpio -> gpio_port_address) -> OTYPER &= ~(0b1 << (gpio -> gpio_num));
		break;
	case GPIO_TYPE_OPEN_DRAIN:
		// OTy = '1'.
		(gpio -> gpio_port_address) -> OTYPER |= (0b1 << (gpio -> gpio_num));
		break;
	default:
		break;
	}
}

/* SET THE OUTPUT SPEED OF A GPIO PIN.
 * @param gpio:			GPIO structure.
 * @param output_speed: Output speed (see enum defined in gpio.h).
 * @return: 			None.
 */
static void GPIO_set_output_speed(const GPIO_pin_t* gpio, GPIO_output_speed_t output_speed) {
	// Set low speed during transition.
	(gpio -> gpio_port_address) -> OSPEEDR &= ~(0b11 << (2 * (gpio -> gpio_num)));
	// Set required bits.
	switch(output_speed) {
	case GPIO_SPEED_LOW:
		// Nothing to do.
		break;
	case GPIO_SPEED_MEDIUM:
		// OSPEEDRy = '01'.
		(gpio -> gpio_port_address) -> OSPEEDR |= (0b1 << (2 * (gpio -> gpio_num)));
		break;
	case GPIO_SPEED_HIGH:
		// OSPEEDRy = '10'.
		(gpio -> gpio_port_address) -> OSPEEDR |= (0b1 << (2 * (gpio -> gpio_num) + 1));
		break;
	case GPIO_SPEED_VERY_HIGH:
		// OSPEEDRy = '11'.
		(gpio -> gpio_port_address) -> OSPEEDR |= (0b11 << (2 * (gpio -> gpio_num)));
		break;
	default:
		break;
	}
}

/* ENABLE OR DISABLE PULL-UP AND PULL-DOWN RESISTORS ON A GPIO PIN.
 * @param gpio:				GPIO structure.
 * @param pull_resistor: 	Resistor configuration (see enum defined in gpio.h).
 * @return: 				None.
 */
static void GPIO_set_pull_resistor(const GPIO_pin_t* gpio, GPIO_pull_resistor_t pull_resistor) {
	// Disable resistors during transition.
	(gpio -> gpio_port_address) -> PUPDR &= ~(0b11 << (2 * (gpio -> gpio_num)));
	// Set required bits.
	switch(pull_resistor) {
	case GPIO_PULL_NONE:
		// Nothing to do.
		break;
	case GPIO_PULL_UP:
		// PUPDRy = '01'.
		(gpio -> gpio_port_address) -> PUPDR |= (0b1 << (2 * (gpio -> gpio_num)));
		break;
	case GPIO_PULL_DOWN:
		// PUPDRy = '10'.
		(gpio -> gpio_port_address) -> PUPDR |= (0b1 << (2 * (gpio -> gpio_num) + 1));
		break;
	default:
		break;
	}
}

/* SELECT THE ALTERNATE FUNCTION OF A GPIO PIN (REQUIRES THE MODE 'GPIO_MODE_ALTERNATE_FUNCTION').
 * @param gpio:			GPIO structure.
 * @param gpio_af_num: 	Alternate function number (0 to 15).
 * @return: 			None.
 */
static void GPIO_set_alternate_function(const GPIO_pin_t* gpio, unsigned int gpio_af_num) {
	// Clamp AF number.
	gpio_af_num &= 0x0F;
	// Select proper register to set.
	if ((gpio -> gpio_num) < GPIO_AFRH_OFFSET) {
		// Set AFRL register: AFRy = 'gpio_af_num'.
		(gpio -> gpio_port_address) -> AFRL &= ~(0b1111 << (4 * (gpio -> gpio_num)));
		(gpio -> gpio_port_address) -> AFRL |= (gpio_af_num << (4 * (gpio -> gpio_num)));
	}
	else {
		// Set AFRH register: AFRy = 'gpio_af_num'.
		(gpio -> gpio_port_address) -> AFRH &= ~(0b1111 << (4 * ((gpio -> gpio_num) - GPIO_AFRH_OFFSET)));
		(gpio -> gpio_port_address) -> AFRH |= (gpio_af_num << (4 * ((gpio -> gpio_num) - GPIO_AFRH_OFFSET)));
	}
}

/*** GPIO functions ***/

/* FUNCTION FOR CONFIGURING A GPIO PIN.
 * @param gpio:				GPIO structure.
 * @param mode: 			Mode (see enum defined in gpio.h).
 * @param output_type:		Output type (see enum defined in gpio.h).
 * @param output_speed: 	Output speed (see enum defined in gpio.h).
 * @param pull_resistor:	Resistor configuration (see enum defined in gpio.h).
 * @param gpio_af_num: 		Alternate function number (0 to 15) if 'GPIO_MODE_ALTERNATE_FUNCTION' mode is selected.
 */
void GPIO_configure(const GPIO_pin_t* gpio, GPIO_mode_t mode, GPIO_output_type_t output_type, GPIO_output_speed_t output_speed, GPIO_pull_resistor_t pull_resistor) {
	GPIO_set_mode(gpio, mode);
	GPIO_set_alternate_function(gpio, (gpio -> gpio_af_num));
	GPIO_set_output_type(gpio, output_type);
	GPIO_set_output_speed(gpio, output_speed);
	GPIO_set_pull_resistor(gpio, pull_resistor);
}

/* CONFIGURE MCU GPIOs.
 * @param: 	None.
 * @return: None.
 */
void GPIO_init(void) {
	// Enable GPIOA, GPIOB and GPIOC clocks.
	RCC -> IOPENR |= (0b111 << 0); // IOPxEN='1'.
	// Configure debug LED pin.
#ifdef DEBUG
	GPIO_configure(&GPIO_LED, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#else
	GPIO_configure(&GPIO_LED, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	// Programming pins.
#ifdef HW2_0
#ifndef DEBUG
	GPIO_configure(&GPIO_SWDIO, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SWCLK, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
#endif
}

/* SET THE STATE OF A GPIO.
 * @param gpio:		GPIO structure.
 * @param state: 	GPIO output state ('0' or '1').
 * @return: 		None.
 */
void __attribute__((optimize("-O0"))) GPIO_write(const GPIO_pin_t* gpio, unsigned char state) {
	// Set bit.
	if (state == 0) {
		(gpio -> gpio_port_address) -> ODR &= ~(0b1 << (gpio -> gpio_num));
	}
	else {
		(gpio -> gpio_port_address) -> ODR |= (0b1 << (gpio -> gpio_num));
	}
}

/* READ THE STATE OF A GPIO.
 * @param gpio:		GPIO structure.
 * @return state: 	Current GPIO input state ('0' or '1').
 */
unsigned char __attribute__((optimize("-O0"))) GPIO_read(const GPIO_pin_t* gpio) {
	// Check mode.
	unsigned char state = 0;
	switch (GPIO_get_mode(gpio)) {
	case GPIO_MODE_INPUT:
		// GPIO configured as input -> read IDR register.
		if ((((gpio -> gpio_port_address) -> IDR) & (0b1 << (gpio -> gpio_num))) != 0) {
			state = 1;
		}
		break;
	case GPIO_MODE_OUTPUT:
		// GPIO configured as output -> read ODR register.
		if ((((gpio -> gpio_port_address) -> ODR) & (0b1 << (gpio -> gpio_num))) != 0) {
			state = 1;
		}
		break;
	default:
		break;
	}
	return state;
}

/* INVERT THE STATE OF A GPIO.
 * @param gpio:	GPIO structure.
 * @return: 	None.
 */
void __attribute__((optimize("-O0"))) GPIO_toggle(const GPIO_pin_t* gpio) {
	// Toggle ODR bit.
	(gpio -> gpio_port_address) -> ODR ^= (0b1 << (gpio -> gpio_num));
}
