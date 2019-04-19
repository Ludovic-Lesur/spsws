/*
 * rcc.c
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#include "rcc.h"

#include "gpio.h"
#include "lptim.h"
#include "lptim_reg.h"
#include "mapping.h"
#include "pwr_reg.h"
#include "rcc_reg.h"
#include "tim.h"
#include "tim_reg.h"

/*** RCC local macros ***/

#define RCC_TIMEOUT_SECONDS		5

/*** RCC local functions ***/

/* PERFORM A 1s. MANUAL DELAY ON THE 2.1MHz MSI CLOCK (RESET CLOCK).
 * @param:	None.
 * @return:	None.
 */
void RCC_DelaySecondMsi(void) {
	unsigned int j = 0;
	for (j=0 ; j<133000 ; j++);
}

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

	/* Unlock back-up registers */
	RCC -> APB1ENR |= (0b1 << 28); // PWREN='1'.
	PWR -> CR |= (0b1 << 8); // Set DBP bit to unlock back-up registers write protection.

	/* Configure TCXO power enable pin as output */
	GPIO_Configure(GPIO_TCXO16_POWER_ENABLE, Output, PushPull, LowSpeed, NoPullUpNoPullDown);
	GPIO_Write(GPIO_TCXO16_POWER_ENABLE, 0);
}

/* CONFIGURE AND USE HSI AS SYSTEM CLOCK (16MHz INTERNAL RC).
 * @param:					None.
 * @return sysclk_on_hsi:	'1' if SYSCLK source was successfully switched to HSI, 0 otherwise.
 */
unsigned char RCC_SwitchToHsi(void) {

	/* Init HSI */
	RCC -> CR |= (0b1 << 0); // Enable HSI (HSI16ON='1').

	/* Wait for HSI to be stable */
	unsigned char sysclk_on_hsi = 0;
	unsigned int second_count = 0;
	while ((((RCC -> CR) & (0b1 << 2)) == 0) && (second_count < RCC_TIMEOUT_SECONDS)) {
		RCC_DelaySecondMsi();
		second_count++; // Wait for HSIRDYF='1' or timeout.
	}

	/* Check timeout */
	if (second_count < RCC_TIMEOUT_SECONDS) {

		/* Switch SYSCLK */
		RCC -> CFGR &= ~(0b11 << 0); // Reset bits 0-1.
		RCC -> CFGR |= (0b01 << 0); // Use HSI as system clock (SW='01').

		/* Wait for clock switch */
		second_count = 0;
		while ((((RCC -> CFGR) & (0b11 << 2)) != (0b01 << 2)) && (second_count < RCC_TIMEOUT_SECONDS)) {
			RCC_DelaySecondMsi();
			second_count++; // Wait for SWS='01' or timeout.
		}

		/* Check timeout */
		if (second_count < RCC_TIMEOUT_SECONDS) {

			/* Disable MSI and HSE */
			RCC -> CR &= ~(0b1 << 8); // Disable MSI (MSION='0').
			RCC -> CR &= ~(0b1 << 16); // Disable HSE (HSEON='0').

			/* Disable 16MHz TCXO power supply */
			GPIO_Write(GPIO_TCXO16_POWER_ENABLE, 0);

			/* Update flag */
			sysclk_on_hsi = 1;
		}
	}

	return sysclk_on_hsi;
}

/* CONFIGURE AND USE HSE AS SYSTEM CLOCK (16MHz TCXO).
 * @param:					None.
 * @return sysclk_on_hse:	'1' if SYSCLK source was successfully switched to HSE (TCXO), 0 otherwise.
 */
unsigned char RCC_SwitchToHse(void) {

	/* Enable 16MHz TCXO */
	GPIO_Write(GPIO_TCXO16_POWER_ENABLE, 1);

	/* Init HSE */
	RCC -> CR |= (0b1 << 18); // Bypass oscillator (HSEBYP='1').
	RCC -> CR |= (0b1 << 16); // Enable HSE (HSEON='1').

	/* Wait for HSE to be stable */
	unsigned char sysclk_on_hse = 0;
	unsigned int second_count = 0;
	while ((((RCC -> CR) & (0b1 << 17)) == 0) && (second_count < RCC_TIMEOUT_SECONDS)) {
		RCC_DelaySecondMsi();
		second_count++; // Wait for HSERDY='1' or timeout.
	}

	/* Check timeout */
	if (second_count < RCC_TIMEOUT_SECONDS) {

		/* Switch SYSCLK */
		RCC -> CFGR &= ~(0b11 << 0); // Reset bits 0-1.
		RCC -> CFGR |= (0b10 << 0); // Use HSE as system clock (SW='10').

		/* Wait for clock switch */
		second_count = 0;
		while ((((RCC -> CFGR) & (0b11 << 2)) != (0b10 << 2)) && (second_count < RCC_TIMEOUT_SECONDS)) {
			RCC_DelaySecondMsi();
			second_count++; // Wait for SWS='10' or timeout.
		}

		/* Check timeout */
		if (second_count < RCC_TIMEOUT_SECONDS) {

			/* Disable MSI and HSI */
			RCC -> CR &= ~(0b1 << 8); // Disable MSI (MSION='0').
			RCC -> CR &= ~(0b1 << 0); // Disable HSI (HSI16ON='0').

			/* Update flag */
			sysclk_on_hse = 1;
		}
	}

	/* Switch TCXO off if any failure occured */
	if (sysclk_on_hse == 0) {
		GPIO_Write(GPIO_TCXO16_POWER_ENABLE, 0);
	}

	return sysclk_on_hse;
}

/* CONFIGURE AND USE LSI AS LOW SPEED OSCILLATOR (32kHz INTERNAL RC).
 * @param:					None.
 * @return lsi_available:	'1' if LSI was successfully started, 0 otherwise.
 */
unsigned char RCC_EnableLsi(void) {

	/* Enable LSI */
	RCC -> CSR |= (0b1 << 0); // LSION='1'.

	/* Wait for LSI to be stable */
	unsigned char lsi_available = 0;
	unsigned int second_count = 0;
	while ((((RCC -> CSR) & (0b1 << 1)) == 0) && (second_count < RCC_TIMEOUT_SECONDS)) {
		RCC_DelaySecondMsi();
		second_count++; // Wait for LSIRDY='1'.
	}

	/* Check timeout */
	if (second_count < RCC_TIMEOUT_SECONDS) {
		// Update flag.
		lsi_available = 1;
	}

	return lsi_available;
}

/* COMPUTE EFFECTIVE LSI OSCILLATOR FREQUENCY.
 * @param:				None.
 * @return lsi_freq_hz:	LSI oscillator frequency in Hz.
 */
unsigned int RCC_GetLsiFrequency(void) {

	/* Local variables */
	unsigned int lsi_freq_hz = 0;

	/* Init and start timers */
	LPTIM1_Init(1);
	TIM21_Init();
	LPTIM1_Start();
	TIM21_Start();

	/* Wait 1 second */
	while (((TIM21 -> SR) & (0b1 << 0)) == 0); // Wait for first overflow.
	TIM21 -> SR &= ~(0b1 << 0); // Clear flag (UIF='0').

	/* Get LSI frequency and stop timers */
	lsi_freq_hz = (LPTIM1 -> CNT);
	LPTIM1_Stop();
	TIM21_Stop();

	/* Check value */
	if (lsi_freq_hz == 0) {
		lsi_freq_hz = 38000;
	}
	return lsi_freq_hz;
}

/* CONFIGURE AND USE LSE AS LOW SPEED OSCILLATOR (32kHz EXTERNAL QUARTZ).
 * @param:					None.
 * @return lsi_available:	'1' if LSE was successfully started, 0 otherwise.
 */
unsigned char RCC_EnableLse(void) {

	/* Configure drive level */
	RCC -> CSR |= (0b11 << 11);

	/* Enable LSE (32.768kHz crystal) */
	RCC -> CSR |= (0b1 << 8); // LSEON='1'.

	/* Wait for LSE to be stable */
	unsigned char lse_available = 0;
	unsigned int second_count = 0;
	while ((((RCC -> CSR) & (0b1 << 9)) == 0) && (second_count < RCC_TIMEOUT_SECONDS)) {
		RCC_DelaySecondMsi();
		second_count++; // Wait for LSERDY='1'.
	}

	/* Check timeout */
	if (second_count < RCC_TIMEOUT_SECONDS) {
		// Update flag.
		lse_available = 1;
	}

	return lse_available;
}
