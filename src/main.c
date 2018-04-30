/*
 * main.c
 *
 *  Created on: 25 apr. 2018
 *      Author: Ludovic
 */

#include "gps.h"
#include "lptim.h"
#include "lpuart.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "gpio_reg.h"

#define MAIN_DEBUG

/* MAIN FUNCTION.
 * @param: 	None.
 * @return: 0.
 */
int main(void) {

#ifdef MAIN_DEBUG

	// Init clock.
	RCC_Init();
	RCC_SwitchToHsi16MHz();

	// Configure PB4 as output.
	RCC -> IOPENR |= (0b1 << 1);
	GPIOB -> MODER &= ~(0b11 << 8); // Reset bits 8-9.
	GPIOB -> MODER |= (0b01 << 8);

	// Init LP timer.
	LPTIM_Init();

	// Init GPS module.
	GPS_Init();

	// Wait for NMEA data.
	while (GPS_Processing() == 0);

	// Switch LPUART and GPS off.
	LPUART_Off();
	GPIOB -> ODR &= ~(0b1 << 5);

	// LED on.
	GPIOB -> ODR |= (0b1 << 4);

#else
	/* Start on multispeed internal oscillator (MSI) */
	RCC_Init();
	RCC_SwitchToMsi65kHz();

	/* Hold power by setting the SHDN pin of the main LDO regulator */
	RCC -> IOPENR |= (0b1 << 2); // Enable GPIOC clock.
	GPIOC -> MODER &= ~(0b11 << 30); // Reset bits 8-9.
	GPIOC -> MODER |= (0b01 << 30); // Configure PC15 as output.
	GPIOC -> ODR |= (0b1 << 15); // Set PC15 to high.

	/* Configure wake-up pin as input */
	RCC -> IOPENR |= (0b1 << 0); // Enable GPIOA clock.
	GPIOA -> MODER &= ~(0b11 << 4); // Configure PA2 as input.

	/* Check hardware timer status */
	/* If the pin is high, last uplink sequence was more than an hour ago -> run new sequence */
	if (((GPIOA -> IDR) & (0b1 << 2)) != 0) {

		/* Switch to external clock (TCXO) */
		RCC_SwitchToHse();

		/* Reset hardware timer */

		/* Initialize LPUART */


	}

	/* Disable main LDO regulator */
	GPIOC -> ODR &= ~(0b1 << 15); // Set PC15 to low.

	/* Configure wake-up pin */

	/* Enter stand-by mode */

#endif

	return (0);
}
