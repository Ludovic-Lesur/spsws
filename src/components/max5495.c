/*
 * max5495.c
 *
 *  Created on: 18 nov. 2018
 *      Author: Ludovic
 */

#include "max5495.h"

#include "gpio_reg.h"
#include "rcc_reg.h"
#include "spi.h"

/*** MAX5495 functions ***/

/* INIT MAX5495 DIGITAL POTENTIOMETER.
 * @param:	None.
 * @return:	None.
 */
void MAX5495_Init(void) {

	/* Init SPI peripheral */
	SPI_Init();

	/* Configure CS GPIO */
	RCC -> IOPENR |= (0b11 << 0);
	GPIOA -> MODER &= ~(0b11 << 2); // Reset bits 2-3.
	GPIOA -> MODER |= (0b01 << 2); // Configure PA1 as output.
	GPIOA -> ODR |= (0b1 << 1); // CS high (idle state).
}


