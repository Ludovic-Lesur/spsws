/*
 * rcc.c
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludovic
 */

#include "rcc.h"
#include "rcc_reg.h"

/*** RCC global variables ***/

unsigned int sysclk_khz = 0;

/*** RCC functions ***/

/* CONFIGURE PERIPHERALs CLOCK PRESCALER AND SOURCES.
 * @param:	None.
 * @return:	None.
 */
void RCC_Init(void) {

	/* Prescalers (HCLK, PCLK1 and PCLK2 must not exceed 32MHz) */
	RCC -> CFGR &= ~(0b1111 << 4); // HCLK = SYSCLK (HPRE='0000').
	RCC -> CFGR &= ~(0b111 << 8); // PCLK1 = HCLK (PPRE1='000').
	RCC -> CFGR &= ~(0b111 << 11); // PCLK2 = HCLK (PPRE2='000').

	/* Peripherals clock source */
	RCC -> CCIPR = 0; // All peripherals clocked via the corresponding APBx line.
}

/* RETURN THE CURRENT SYSTEM CLOCK FREQUENCY IN KHZ.
 * @param:				None.
 * @return sysclk_khz:	Current system clock frequency in kHz.
 */
unsigned int RCC_GetSysclkKhz(void) {
	return sysclk_khz;
}

/* CONFIGURE AND USE MSI RANGE 1 AS SYSTEM CLOCK (131.072kHz).
 * @param:	None.
 * @return:	None.
 */
void RCC_SwitchToMsi131kHz(void) {

	/* Init MSI */
	RCC -> ICSCR &= ~(0b111 << 13); // Reset bits 13-15.
	RCC -> ICSCR |= (0b001 << 13); // Set MSI frequency to range 1 = 131.072kHz (MSIRANGE='001').
	RCC -> CR |= (0b1 << 8); // Enable MSI (MSION='1').
	while (((RCC -> CR) & (0b1 << 9)) == 0); // Wait for MSI to be stable (MSIRDY='1').
	RCC -> CFGR &= ~(0b11 << 0); // Use MSI as system clock (SW='00').
	while (((RCC -> CFGR) & (0b11 << 2)) != (0b00 << 2)); // Wait for clock switch (SWS='00').

	/* Disable HSI and HSE */
	RCC -> CR &= ~(0b1 << 0); // Disable HSI (HSI16ON='0').
	RCC -> CR &= ~(0b1 << 16); // Disable HSE (HSEON='0').

	/* Update system clock */
	sysclk_khz = 131;
}

/* CONFIGURE AND USE HSI AS SYSTEM CLOCK (16MHz).
 * @param:	None.
 * @return:	None.
 */
void RCC_SwitchToHsi16MHz(void) {

	/* Init HSI */
	RCC -> CR |= (0b1 << 0); // Enable HSI (HSI16ON='1').
	while (((RCC -> CR) & (0b1 << 2)) == 0); // Wait for HSI to be stable (HSIRDYF='1').
	RCC -> CFGR &= ~(0b11 << 0); // Reset bits 0-1.
	RCC -> CFGR |= (0b01 << 0); // Use HSI as system clock (SW='01').
	while (((RCC -> CFGR) & (0b11 << 2)) != (0b01 << 2)); // Wait for clock switch (SWS='01').

	/* Disable MSI and HSE */
	RCC -> CR &= ~(0b1 << 8); // Disable MSI (MSION='0').
	RCC -> CR &= ~(0b1 << 16); // Disable HSE (HSEON='0').

	/* Update system clock */
	sysclk_khz = 16000;
}

/* CONFIGURE AND USE HSE AS SYSTEM CLOCK (...MHz).
 * @param:	None.
 * @return:	None.
 */
void RCC_SwitchToHse(void) {

	/* Init HSE */
	RCC -> CR |= (0b1 << 16); // Enable HSE (HSEON='1').
	while (((RCC -> CR) & (0b1 << 17)) == 0); // Wait for HSE to be stable (HSERDY='1').
	RCC -> CFGR &= ~(0b11 << 0); // Reset bits 0-1.
	RCC -> CFGR |= (0b10 << 0); // Use HSE as system clock (SW='10').
	while (((RCC -> CFGR) & (0b11 << 2)) != (0b10 << 2)); // Wait for clock switch (SWS='10').

	/* Disable MSI and HSI */
	RCC -> CR &= ~(0b1 << 8); // Disable MSI (MSION='0').
	RCC -> CR &= ~(0b1 << 0); // Disable HSI (HSI16ON='0').

	/* Update system clock */
	sysclk_khz = 0;
}
