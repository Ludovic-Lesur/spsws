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

/*** GPIO local macros ***/

#define GPIO_PER_PORT 	16 // Each port_address (A to K) has 16 GPIO.
#define AF_PER_GPIO 	16 // Each GPIO has 16 alternate functions.
#define AFRH_OFFSET 	8 // Limit between AFRL and AFRH registers.

/*** GPIO local functions ***/

/* SET THE MODE OF A GPIO PIN.
 * @param gpio:	GPIO structure.
 * @param mode: Desired mode ('Input', 'Output', 'AlternateFunction' or 'Analog').
 * @return: 	None.
 */
void GPIO_SetMode(GPIO gpio, GPIO_Mode mode) {
	// Ensure GPIO exists.
	if ((gpio.num >= 0) && (gpio.num < GPIO_PER_PORT)) {
		switch(mode) {
		case Input:
			// MODERy = '00'.
			gpio.port_address -> MODER &= ~(0b1 << (2*gpio.num));
			gpio.port_address -> MODER &= ~(0b1 << (2*gpio.num+1));
			break;
		case Output:
			// MODERy = '01'.
			gpio.port_address -> MODER |= (0b1 << (2*gpio.num));
			gpio.port_address -> MODER &= ~(0b1 << (2*gpio.num+1));
			break;
		case Analog:
			// MODERy = '11'.
			gpio.port_address -> MODER |= (0b1 << (2*gpio.num));
			gpio.port_address -> MODER |= (0b1 << (2*gpio.num+1));
			break;
		case AlternateFunction:
			// MODERy = '10'.
			gpio.port_address -> MODER &= ~(0b1 << (2*gpio.num));
			gpio.port_address -> MODER |= (0b1 << (2*gpio.num+1));
			break;
		default:
			break;
		}
	}
}

/* GET THE MODE OF A GPIO PIN.
 * @param gpio:			GPIO structure.
 * @return gpioMode: 	Current mode of the  GPIO ('Input', 'Output', 'AlternateFunction' or 'Analog').
 */
GPIO_Mode GPIO_GetMode(GPIO gpio) {
	unsigned char bit0 = ((gpio.port_address -> MODER) & (0b1 << (2*gpio.num))) >> (2*gpio.num);
	unsigned char bit1 = ((gpio.port_address -> MODER) & (0b1 << (2*gpio.num+1))) >> (2*gpio.num+1);
	GPIO_Mode gpioMode = (bit1 << 1) + bit0;
	return gpioMode;
}

/* SET THE OUTPUT TYPE OF A GPIO PIN.
 * @param gpio:			GPIO structure.
 * @param outputType: 	Desired output ('PushPull' or 'OpenDrain').
 * @return: 			None.
 */
void GPIO_SetOutputType(GPIO gpio, GPIO_OutputType output_type) {
	// Ensure GPIO exists.
	if ((gpio.num >= 0) && (gpio.num < GPIO_PER_PORT)) {
		switch(output_type) {
		case PushPull:
			// OTy = '0'.
			gpio.port_address -> OTYPER &= ~(0b1 << gpio.num);
			break;
		case OpenDrain:
			// OTy = '1'.
			gpio.port_address -> OTYPER |= (0b1 << gpio.num);
			break;
		default:
			break;
		}
	}
}

/* SET THE OUTPUT SPEED OF A GPIO PIN.
 * @param gpio:			GPIO structure.
 * @param outputSpeed: 	Desired output speed ('LowSpeed', 'MediumSpeed', 'HighSpeed' or 'VeryHighSpeed').
 * @return: 			None.
 */
void GPIO_SetOutputSpeed(GPIO gpio, GPIO_OutputSpeed output_speed) {
	// Ensure GPIO exists.
	if ((gpio.num >= 0) && (gpio.num < GPIO_PER_PORT)) {
		switch(output_speed) {
		case LowSpeed:
			// OSPEEDRy = '00'.
			gpio.port_address -> OSPEEDR &= ~(0b1 << (2*gpio.num));
			gpio.port_address -> OSPEEDR &= ~(0b1 << (2*gpio.num+1));
			break;
		case MediumSpeed:
			// OSPEEDRy = '01'.
			gpio.port_address -> OSPEEDR |= (0b1 << (2*gpio.num));
			gpio.port_address -> OSPEEDR &= ~(0b1 << (2*gpio.num+1));
			break;
		case HighSpeed:
			// OSPEEDRy = '10'.
			gpio.port_address -> OSPEEDR &= ~(0b1 << (2*gpio.num));
			gpio.port_address -> OSPEEDR |= (0b1 << (2*gpio.num+1));
			break;
		case VeryHighSpeed:
			// OSPEEDRy = '11'.
			gpio.port_address -> OSPEEDR |= (0b1 << (2*gpio.num));
			gpio.port_address -> OSPEEDR |= (0b1 << (2*gpio.num+1));
			break;
		default:
			break;
		}
	}
}

/* ENABLE OR DISABLE PULL-UP AND PULL-DOWN RESISTORS ON A GPIO PIN.
 * @param gpio:				GPIO structure.
 * @param pullResistor: 	Desired configuration ('NoPullUpNoPullDown', 'PullUp', or 'PullDown').
 * @return: 				None.
 */
void GPIO_SetPullUpPullDown(GPIO gpio, GPIO_PullResistor pull_resistor) {
	// Ensure GPIO exists.
	if ((gpio.num >= 0) && (gpio.num < GPIO_PER_PORT)) {
		switch(pull_resistor) {
		case NoPullUpNoPullDown:
			// PUPDRy = '00'.
			gpio.port_address -> PUPDR &= ~(0b1 << (2*gpio.num));
			gpio.port_address -> PUPDR &= ~(0b1 << (2*gpio.num+1));
			break;
		case PullUp:
			// PUPDRy = '01'.
			gpio.port_address -> PUPDR |= (0b1 << (2*gpio.num));
			gpio.port_address -> PUPDR &= ~(0b1 << (2*gpio.num+1));
			break;
		case PullDown:
			// PUPDRy = '10'.
			gpio.port_address -> PUPDR &= ~(0b1 << (2*gpio.num));
			gpio.port_address -> PUPDR |= (0b1 << (2*gpio.num+1));
			break;
		default:
			break;
		}
	}
}

/* SELECT THE ALTERNATE FUNCTION OF A GPIO PIN (REQUIRES THE MODE 'AlternateFunction').
 * @param gpio:		GPIO structure.
 * @param af_num: 	Alternate function number (0 to 15).
 * @return: 		None.
 */
void GPIO_SetAlternateFunction(GPIO gpio, unsigned int af_num) {
	// Ensure alternate function exists.
	if ((af_num >= 0) && (af_num < AF_PER_GPIO)) {
		unsigned int i = 0;
		// Select proper register to set.
		if ((gpio.num >= 0) && (gpio.num < AFRH_OFFSET)) {
			// Set AFRL register: AFRy = 'af_num'.
			for (i=0 ; i<4 ; i++) {
				if (af_num & (0b1 << i)) {
					// Bit = '1'.
					gpio.port_address -> AFRL |= (0b1 << (4*gpio.num+i));
				}
				else {
					// Bit = '0'.
					gpio.port_address -> AFRL &= ~(0b1 << (4*gpio.num+i));
				}
			}
		}
		else {
			if ((gpio.num >= AFRH_OFFSET) && (gpio.num < GPIO_PER_PORT)) {
				// Set AFRH register: AFRy = 'af_num'.
				for (i=0 ; i<4 ; i++) {
					if (af_num & (0b1 << i)) {
						// Bit = '1'.
						gpio.port_address -> AFRH |= (0b1 << (4*(gpio.num-AFRH_OFFSET)+i));
					}
					else {
						// Bit = '0'.
						gpio.port_address -> AFRH &= ~(0b1 << (4*(gpio.num-AFRH_OFFSET)+i));
					}
				}
			}
		}
	}
}

/*** GPIO functions ***/

/* FUNCTION FOR CONFIGURING A GPIO PIN.
 * @param gpio:			GPIO structure.
 * @param mode: 		Desired mode ('Input', 'Output', 'AlternateFunction' or 'Analog').
 * @param outputType:	Desired output ('PushPull' or 'OpenDrain').
 * @param outputSpeed: 	Desired output speed ('0Speed', 'MediumSpeed', '1Speed' or 'Very1Speed').
 * @param pullResistor: Desired configuration ('NoPullUpNoPullDown', 'PullUp', or 'PullDown').
 * @param af_num: 		Alternate function number (0 to 15) if 'AlternateFunction' mode is selected.
 */
void GPIO_Configure(GPIO gpio, GPIO_Mode mode, GPIO_OutputType output_type, GPIO_OutputSpeed output_speed, GPIO_PullResistor pull_resistor) {
	GPIO_SetMode(gpio, mode);
	if (mode == AlternateFunction) {
		GPIO_SetAlternateFunction(gpio, gpio.af_num);
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
#ifndef IM_HWT
	GPIO_Configure(GPIO_LED, Output, PushPull, LowSpeed, NoPullUpNoPullDown);
#endif

	/* Others GPIOs are configured in their corresponding peripheral or applicative driver */
}

/* SET THE STATE OF A GPIO.
 * @param gpio:		GPIO structure.
 * @param state: 	Desired state of the pin ('0' or '1').
 * @return: 		None.
 */
void GPIO_Write(GPIO gpio, unsigned char state) {
	// Ensure GPIO exists.
	if ((gpio.num >= 0) && (gpio.num < GPIO_PER_PORT)) {
		if (state) {
			gpio.port_address -> ODR |= (0b1 << gpio.num);
		}
		else {
			gpio.port_address -> ODR &= ~(0b1 << gpio.num);
		}
	}
}

/* READ THE STATE OF A GPIO.
 * @param gpio:		GPIO structure.
 * @return state: 	GPIO state ('0' or '1').
 */
unsigned char GPIO_Read(GPIO gpio) {
	unsigned char state = 0;
	if ((gpio.num >= 0) && (gpio.num < GPIO_PER_PORT)) {
		switch (GPIO_GetMode(gpio)) {
		case Input:
			// GPIO configured as input -> read IDR register.
			if (((gpio.port_address -> IDR) & (0b1 << gpio.num)) != 0) {
				state = 1;
			}
			break;
		case Output:
			// GPIO configured as output -> read ODR register.
			if (((gpio.port_address -> ODR) & (0b1 << gpio.num)) != 0) {
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
void GPIO_Toggle(GPIO gpio) {
	// Ensure GPIO exists.
	if ((gpio.num >= 0) && (gpio.num < GPIO_PER_PORT)) {
		if (GPIO_Read(gpio) == 0) {
			GPIO_Write(gpio, 1);
		}
		else {
			GPIO_Write(gpio, 0);
		}
	}
}
