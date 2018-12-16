/*
 * rcc.c
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludovic
 */

#include "rcc.h"

#include "gpio.h"
#include "mapping.h"
#include "rcc_reg.h"

/*** RCC functions ***/

/* CONFIGURE PERIPHERALs CLOCK PRESCALER AND SOURCES.
 * @param:	None.
 * @return:	None.
 */
void RCC_Init(void) {

	/* Prescalers (HCLK, PCLK1 and PCLK2 must not exceed 32MHz) */
	RCC -> CFGR &= ~(0b1111 << 4); // HCLK = SYSCLK = 16MHz (HPRE='0000').
	RCC -> CFGR &= ~(0b111 << 8); // PCLK1 = HCLK = 16MHz (PPRE1='000').
	RCC -> CFGR &= ~(0b111 << 11); // PCLK2 = HCLK = 16MHz (PPRE2='000').

	/* Peripherals clock source */
	RCC -> CCIPR = 0; // All peripherals clocked via the corresponding APBx line.

	/* Configure TCXO power enable pin (PB8) as output */
	GPIO_Configure(GPIO_TCXO_POWER_ENABLE, Output, PushPull, LowSpeed, NoPullUpNoPullDown);
	GPIO_Write(GPIO_TCXO_POWER_ENABLE, 0);
}

/* CONFIGURE AND USE HSI AS SYSTEM CLOCK (16 MHz internal RC).
 * @param:	None.
 * @return:	None.
 */
void RCC_SwitchToInternal16MHz(void) {

	/* Init HSI */
	RCC -> CR |= (0b1 << 0); // Enable HSI (HSI16ON='1').
	while (((RCC -> CR) & (0b1 << 2)) == 0); // Wait for HSI to be stable (HSIRDYF='1').
	RCC -> CFGR &= ~(0b11 << 0); // Reset bits 0-1.
	RCC -> CFGR |= (0b01 << 0); // Use HSI as system clock (SW='01').
	while (((RCC -> CFGR) & (0b11 << 2)) != (0b01 << 2)); // Wait for clock switch (SWS='01').

	/* Disable MSI and HSE */
	RCC -> CR &= ~(0b1 << 8); // Disable MSI (MSION='0').
	RCC -> CR &= ~(0b1 << 16); // Disable HSE (HSEON='0').

	/* Disable TCXO power supply */
	GPIO_Write(GPIO_TCXO_POWER_ENABLE, 0);
}

/* CONFIGURE AND USE HSE AS SYSTEM CLOCK (16 MHz TCXO).
 * @param:	None.
 * @return:	None.
 */
void RCC_SwitchToTcxo16MHz(void) {

	/* Enable TCXO power supply (PB8) */
	GPIO_Write(GPIO_TCXO_POWER_ENABLE, 1);

	/* Init HSE */
	RCC -> CR |= (0b1 << 18); // Bypass oscillator (HSEBYP='1').
	RCC -> CR |= (0b1 << 16); // Enable HSE (HSEON='1').
	while (((RCC -> CR) & (0b1 << 17)) == 0); // Wait for HSE to be stable (HSERDY='1').
	RCC -> CFGR &= ~(0b11 << 0); // Reset bits 0-1.
	RCC -> CFGR |= (0b10 << 0); // Use HSE as system clock (SW='10').
	while (((RCC -> CFGR) & (0b11 << 2)) != (0b10 << 2)); // Wait for clock switch (SWS='10').

	/* Disable MSI and HSI */
	RCC -> CR &= ~(0b1 << 8); // Disable MSI (MSION='0').
	RCC -> CR &= ~(0b1 << 0); // Disable HSI (HSI16ON='0').
}
