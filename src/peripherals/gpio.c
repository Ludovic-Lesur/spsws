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

#define GPIO_PER_PORT 	16 	// Each gpio_port_address (A to K) has 16 GPIO.
#define AF_PER_GPIO 	16 	// Each GPIO has 16 alternate functions.
#define AFRH_OFFSET 	8 	// Limit between AFRL and AFRH registers.

/*** GPIO local functions ***/

/* SET THE MODE OF A GPIO PIN.
 * @param gpio:	GPIO structure.
 * @param mode: Desired mode ('GPIO_MODE_INPUT', 'GPIO_MODE_OUTPUT', 'AlternateFunction' or 'GPIO_MODE_ANALOG').
 * @return: 	None.
 */
void GPIO_SetMode(const GPIO* gpio, GPIO_Mode mode) {
	// Ensure GPIO exists.
	if (((gpio -> gpio_num) >= 0) && ((gpio -> gpio_num) < GPIO_PER_PORT)) {
		switch(mode) {
		case GPIO_MODE_INPUT:
			// MODERy = '00'.
			(gpio -> gpio_port_address) -> MODER &= ~(0b1 << (2*(gpio -> gpio_num)));
			(gpio -> gpio_port_address) -> MODER &= ~(0b1 << (2*(gpio -> gpio_num)+1));
			break;
		case GPIO_MODE_OUTPUT:
			// MODERy = '01'.
			(gpio -> gpio_port_address) -> MODER |= (0b1 << (2*(gpio -> gpio_num)));
			(gpio -> gpio_port_address) -> MODER &= ~(0b1 << (2*(gpio -> gpio_num)+1));
			break;
		case GPIO_MODE_ANALOG:
			// MODERy = '11'.
			(gpio -> gpio_port_address) -> MODER |= (0b1 << (2*(gpio -> gpio_num)));
			(gpio -> gpio_port_address) -> MODER |= (0b1 << (2*(gpio -> gpio_num)+1));
			break;
		case GPIO_MODE_ALTERNATE_FUNCTION:
			// MODERy = '10'.
			(gpio -> gpio_port_address) -> MODER &= ~(0b1 << (2*(gpio -> gpio_num)));
			(gpio -> gpio_port_address) -> MODER |= (0b1 << (2*(gpio -> gpio_num)+1));
			break;
		default:
			break;
		}
	}
}

/* GET THE MODE OF A GPIO PIN.
 * @param gpio:			GPIO structure.
 * @return gpioMode: 	Current mode of the  GPIO ('GPIO_MODE_INPUT', 'GPIO_MODE_OUTPUT', 'AlternateFunction' or 'GPIO_MODE_ANALOG').
 */
GPIO_Mode GPIO_GetMode(const GPIO* gpio) {
	unsigned char bit0 = (((gpio -> gpio_port_address) -> MODER) & (0b1 << (2*(gpio -> gpio_num)))) >> (2*(gpio -> gpio_num));
	unsigned char bit1 = (((gpio -> gpio_port_address) -> MODER) & (0b1 << (2*(gpio -> gpio_num)+1))) >> (2*(gpio -> gpio_num)+1);
	GPIO_Mode gpioMode = (bit1 << 1) + bit0;
	return gpioMode;
}

/* SET THE OUTPUT TYPE OF A GPIO PIN.
 * @param gpio:			GPIO structure.
 * @param outputType: 	Desired output ('PushPull' or 'GPIO_TYPE_OPEN_DRAIN').
 * @return: 			None.
 */
void GPIO_SetOutputType(const GPIO* gpio, GPIO_OutputType output_type) {
	// Ensure GPIO exists.
	if (((gpio -> gpio_num) >= 0) && ((gpio -> gpio_num) < GPIO_PER_PORT)) {
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
}

/* SET THE OUTPUT SPEED OF A GPIO PIN.
 * @param gpio:			GPIO structure.
 * @param outputSpeed: 	Desired output speed ('GPIO_SPEED_LOW', 'MediumSpeed', 'HighSpeed' or 'VeryHighSpeed').
 * @return: 			None.
 */
void GPIO_SetOutputSpeed(const GPIO* gpio, GPIO_OutputSpeed output_speed) {
	// Ensure GPIO exists.
	if (((gpio -> gpio_num) >= 0) && ((gpio -> gpio_num) < GPIO_PER_PORT)) {
		switch(output_speed) {
		case GPIO_SPEED_LOW:
			// OSPEEDRy = '00'.
			(gpio -> gpio_port_address) -> OSPEEDR &= ~(0b1 << (2*(gpio -> gpio_num)));
			(gpio -> gpio_port_address) -> OSPEEDR &= ~(0b1 << (2*(gpio -> gpio_num)+1));
			break;
		case GPIO_SPEED_MEDIUM:
			// OSPEEDRy = '01'.
			(gpio -> gpio_port_address) -> OSPEEDR |= (0b1 << (2*(gpio -> gpio_num)));
			(gpio -> gpio_port_address) -> OSPEEDR &= ~(0b1 << (2*(gpio -> gpio_num)+1));
			break;
		case GPIO_SPEED_HIGH:
			// OSPEEDRy = '10'.
			(gpio -> gpio_port_address) -> OSPEEDR &= ~(0b1 << (2*(gpio -> gpio_num)));
			(gpio -> gpio_port_address) -> OSPEEDR |= (0b1 << (2*(gpio -> gpio_num)+1));
			break;
		case GPIO_SPEED_VERY_HIGH:
			// OSPEEDRy = '11'.
			(gpio -> gpio_port_address) -> OSPEEDR |= (0b1 << (2*(gpio -> gpio_num)));
			(gpio -> gpio_port_address) -> OSPEEDR |= (0b1 << (2*(gpio -> gpio_num)+1));
			break;
		default:
			break;
		}
	}
}

/* ENABLE OR DISABLE PULL-UP AND PULL-DOWN RESISTORS ON A GPIO PIN.
 * @param gpio:				GPIO structure.
 * @param pullResistor: 	Desired configuration ('GPIO_PULL_NONE', 'PullUp', or 'PullDown').
 * @return: 				None.
 */
void GPIO_SetPullUpPullDown(const GPIO* gpio, GPIO_PullResistor pull_resistor) {
	// Ensure GPIO exists.
	if (((gpio -> gpio_num) >= 0) && ((gpio -> gpio_num) < GPIO_PER_PORT)) {
		switch(pull_resistor) {
		case GPIO_PULL_NONE:
			// PUPDRy = '00'.
			(gpio -> gpio_port_address) -> PUPDR &= ~(0b1 << (2*(gpio -> gpio_num)));
			(gpio -> gpio_port_address) -> PUPDR &= ~(0b1 << (2*(gpio -> gpio_num)+1));
			break;
		case GPIO_PULL_UP:
			// PUPDRy = '01'.
			(gpio -> gpio_port_address) -> PUPDR |= (0b1 << (2*(gpio -> gpio_num)));
			(gpio -> gpio_port_address) -> PUPDR &= ~(0b1 << (2*(gpio -> gpio_num)+1));
			break;
		case GPIO_PULL_DOWN:
			// PUPDRy = '10'.
			(gpio -> gpio_port_address) -> PUPDR &= ~(0b1 << (2*(gpio -> gpio_num)));
			(gpio -> gpio_port_address) -> PUPDR |= (0b1 << (2*(gpio -> gpio_num)+1));
			break;
		default:
			break;
		}
	}
}

/* SELECT THE ALTERNATE FUNCTION OF A GPIO PIN (REQUIRES THE MODE 'AlternateFunction').
 * @param gpio:		GPIO structure.
 * @param gpio_af_num: 	Alternate function gpio_number (0 to 15).
 * @return: 		None.
 */
void GPIO_SetAlternateFunction(const GPIO* gpio, unsigned int gpio_af_num) {
	// Ensure alternate function exists.
	if ((gpio_af_num >= 0) && (gpio_af_num < AF_PER_GPIO)) {
		unsigned int i = 0;
		// Select proper register to set.
		if (((gpio -> gpio_num) >= 0) && ((gpio -> gpio_num) < AFRH_OFFSET)) {
			// Set AFRL register: AFRy = 'gpio_af_num'.
			for (i=0 ; i<4 ; i++) {
				if (gpio_af_num & (0b1 << i)) {
					// Bit = '1'.
					(gpio -> gpio_port_address) -> AFRL |= (0b1 << (4*(gpio -> gpio_num)+i));
				}
				else {
					// Bit = '0'.
					(gpio -> gpio_port_address) -> AFRL &= ~(0b1 << (4*(gpio -> gpio_num)+i));
				}
			}
		}
		else {
			if (((gpio -> gpio_num) >= AFRH_OFFSET) && ((gpio -> gpio_num) < GPIO_PER_PORT)) {
				// Set AFRH register: AFRy = 'gpio_af_num'.
				for (i=0 ; i<4 ; i++) {
					if (gpio_af_num & (0b1 << i)) {
						// Bit = '1'.
						(gpio -> gpio_port_address) -> AFRH |= (0b1 << (4*((gpio -> gpio_num)-AFRH_OFFSET)+i));
					}
					else {
						// Bit = '0'.
						(gpio -> gpio_port_address) -> AFRH &= ~(0b1 << (4*((gpio -> gpio_num)-AFRH_OFFSET)+i));
					}
				}
			}
		}
	}
}

/*** GPIO functions ***/

/* FUNCTION FOR CONFIGURING A GPIO PIN.
 * @param gpio:			GPIO structure.
 * @param mode: 		Desired mode ('GPIO_MODE_INPUT', 'GPIO_MODE_OUTPUT', 'AlternateFunction' or 'GPIO_MODE_ANALOG').
 * @param outputType:	Desired output ('PushPull' or 'GPIO_TYPE_OPEN_DRAIN').
 * @param outputSpeed: 	Desired output speed ('0Speed', 'MediumSpeed', '1Speed' or 'Very1Speed').
 * @param pullResistor: Desired configuration ('GPIO_PULL_NONE', 'PullUp', or 'PullDown').
 * @param gpio_af_num: 		Alternate function gpio_number (0 to 15) if 'AlternateFunction' mode is selected.
 */
void GPIO_Configure(const GPIO* gpio, GPIO_Mode mode, GPIO_OutputType output_type, GPIO_OutputSpeed output_speed, GPIO_PullResistor pull_resistor) {
	GPIO_SetMode(gpio, mode);
	if (mode == GPIO_MODE_ALTERNATE_FUNCTION) {
		GPIO_SetAlternateFunction(gpio, (gpio -> gpio_af_num));
	}
	GPIO_SetOutputType(gpio, output_type);
	GPIO_SetOutputSpeed(gpio, output_speed);
	GPIO_SetPullUpPullDown(gpio, pull_resistor);
}

/* CONFIGURE MCU GPIOs.
 * @param: 	None.
 * @return: None.
 */
void GPIO_Init(void) {

	/* Enable GPIOA, GPIOB and GPIOC clocks */
	RCC -> IOPENR |= (0b111 << 0); // IOPxEN='1'.

	/* Configure standalone GPIOs */
	// LED.
#ifndef USE_HWT
#ifdef DEBUG
	GPIO_Configure(&GPIO_LED, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#else
	GPIO_Configure(&GPIO_LED, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
#endif
	// Programming pins.
#ifdef HW2_0
#ifndef DEBUG
	GPIO_Configure(&GPIO_SWDIO, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_SWCLK, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
#endif
	// Digital I/O.
#ifdef HW1_0
#ifndef USE_HWT
	GPIO_Configure(&GPIO_DIO0, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
#ifndef USE_MAX11136_EOC
	GPIO_Configure(&GPIO_DIO1, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	GPIO_Configure(&GPIO_DIO2, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#ifndef USE_SX1232_DIOX
	GPIO_Configure(&GPIO_DIO3, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
#endif
#ifdef HW2_0
	GPIO_Configure(&GPIO_DIO0, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_DIO1, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_DIO2, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_DIO3, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_DIO4, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif

	/* Others GPIOs are configured in their corresponding peripheral or applicative driver */
}

/* SET THE STATE OF A GPIO.
 * @param gpio:		GPIO structure.
 * @param state: 	Desired state of the pin ('0' or '1').
 * @return: 		None.
 */
void GPIO_Write(const GPIO* gpio, unsigned char state) {
	// Ensure GPIO exists.
	if (((gpio -> gpio_num) >= 0) && ((gpio -> gpio_num) < GPIO_PER_PORT)) {
		if (state) {
			(gpio -> gpio_port_address) -> ODR |= (0b1 << (gpio -> gpio_num));
		}
		else {
			(gpio -> gpio_port_address) -> ODR &= ~(0b1 << (gpio -> gpio_num));
		}
	}
}

/* READ THE STATE OF A GPIO.
 * @param gpio:		GPIO structure.
 * @return state: 	GPIO state ('0' or '1').
 */
unsigned char GPIO_Read(const GPIO* gpio) {
	unsigned char state = 0;
	if (((gpio -> gpio_num) >= 0) && ((gpio -> gpio_num) < GPIO_PER_PORT)) {
		switch (GPIO_GetMode(gpio)) {
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
	}
	return state;
}

/* INVERT THE STATE OF A GPIO.
 * @param gpio:	GPIO structure.
 * @return: 	None.
 */
void GPIO_Toggle(const GPIO* gpio) {
	// Ensure GPIO exists.
	if (((gpio -> gpio_num) >= 0) && ((gpio -> gpio_num) < GPIO_PER_PORT)) {
		if (GPIO_Read(gpio) == 0) {
			GPIO_Write(gpio, 1);
		}
		else {
			GPIO_Write(gpio, 0);
		}
	}
}
