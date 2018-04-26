/*
 * main.c
 *
 *  Created on: 25 apr. 2018
 *      Author: Ludovic
 */

#include "rcc_reg.h"
#include "gpio_reg.h"

/* MAIN FUNCTION.
 * @param: 	None.
 * @return: 0.
 */
int main(void) {

	// Enable GPIOB clock.
	RCC -> IOPENR |= (0b1 << 1);

	// Configure PB4 as output.
	GPIOB -> MODER &= ~(0b11 << 8); // Reset bits 8-9.
	GPIOB -> MODER |= (0b01 << 8);

	unsigned int i = 0;

	while(1) {
		// LED on.
		GPIOB -> ODR |= (0b1 << 4);
		for (i=0 ; i<100000 ; i++);
		// LED off.
		GPIOB -> ODR &= ~(0b1 << 4);
		for (i=0 ; i<100000 ; i++);
	}

	return (0);
}
