/*
 * main.c
 *
 *  Created on: 25 apr. 2018
 *      Author: Ludovic
 */

#include "adc.h"
#include "gpio_reg.h"
#include "gps.h"
#include "lpuart.h"
#include "pwr.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "scb_reg.h"
#include "tim.h"

#define MAIN_DEBUG

/* MAIN FUNCTION.
 * @param: 	None.
 * @return: 0.
 */
int main(void) {

	/* Reset deep sleep flag on wake-up */
	SCB -> SCR &= ~(0b1 << 2); // SLEEPDEEP='0'.

#ifdef MAIN_DEBUG

	// Init clock.
	RCC_Init();
	RCC_SwitchToHsi16MHz();

	// Init time.
	TIM_TimeInit();

	// Configure PB4 as output.
	RCC -> IOPENR |= (0b1 << 1);
	GPIOB -> MODER &= ~(0b11 << 8); // Reset bits 8-9.
	GPIOB -> MODER |= (0b01 << 8);

	// Wait for NMEA data.
	GPIOB -> ODR |= (0b1 << 4); // LED on.
	unsigned char sigfox_gps_data[11] = {0};
	while (GPS_Processing(sigfox_gps_data, 11) == 0);

	// Init ADC.
	ADC_Init();

	// Get actual supply voltage.
	unsigned int mcu_vdd;
	ADC_GetMcuVddMv(&mcu_vdd);

	// Get MCU internal temperature.
	int mcu_temp;
	ADC_GetMcuTempDegrees(&mcu_temp);

	// LED off.
	GPIOB -> ODR &= ~(0b1 << 4);

	// Enter standby mode.
	PWR_EnterStandbyMode();

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
	}

	/* Disable main LDO regulator */
	GPIOC -> ODR &= ~(0b1 << 15); // Set PC15 to low.

	/* Enter stand-by mode */
	PWR_EnterStandbyMode();

#endif

	return (0);
}
