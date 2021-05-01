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
#include "nvic.h"
#include "pwr_reg.h"
#include "rcc_reg.h"
#include "scb_reg.h"
#include "tim.h"
#include "tim_reg.h"

/*** RCC local macros ***/

#define RCC_TIMEOUT_COUNT	10

/*** RCC local global variables ***/

static unsigned int rcc_sysclk_khz;

/*** RCC local functions ***/

/* PERFORM A MANUAL DELAY.
 * @param:	None.
 * @return:	None.
 */
static void RCC_Delay(void) {
	unsigned int j = 0;
	unsigned int loop_count = (19 * rcc_sysclk_khz) / 3; // Value for 100ms.
	for (j=0 ; j<loop_count ; j++) {
		// Poll a bit always read as '0'.
		// This is required to avoid for loop removal by compiler (size optimization for HW1.0).
		if (((RCC -> CR) & (0b1 << 24)) != 0) {
			break;
		}
	}
}

/*** RCC functions ***/

/* CONFIGURE PERIPHERALs CLOCK PRESCALER AND SOURCES.
 * @param:	None.
 * @return:	None.
 */
void RCC_Init(void) {
	// Prescalers (HCLK, PCLK1 and PCLK2 must not exceed 32MHz).
	RCC -> CFGR &= ~(0b1111 << 4); // HCLK = SYSCLK = 16MHz (HPRE='0000').
	RCC -> CFGR &= ~(0b111 << 8); // PCLK1 = HCLK = 16MHz (PPRE1='000').
	RCC -> CFGR &= ~(0b111 << 11); // PCLK2 = HCLK = 16MHz (PPRE2='000').
	// Peripherals clock source.
	RCC -> CCIPR &= 0xFFF0C3F0; // All peripherals clocked via the corresponding APBx line.
	// Unlock back-up registers.
	RCC -> APB1ENR |= (0b1 << 28); // PWREN='1'.
	PWR -> CR |= (0b1 << 8); // Set DBP bit to unlock back-up registers write protection.
	// Reset clock is MSI 2.1MHz.
	rcc_sysclk_khz = 2100;
}

/* ENABLE RCC GPIO.
 * @param:	None.
 * @return:	None.
 */
void RCC_EnableGpio(void) {
	// Configure TCXO power enable pin as output.
	GPIO_Configure(&GPIO_TCXO16_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_TCXO16_POWER_ENABLE, 0);
}

/* DISABLE RCC GPIO.
 * @param:	None.
 * @return:	None.
 */
void RCC_DisableGpio(void) {
	// Disable TCXO power control pin.
	GPIO_Configure(&GPIO_TCXO16_POWER_ENABLE, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/* RETURN THE CURRENT SYSTEM CLOCK FREQUENCY.
 * @param:					None.
 * @return rcc_sysclk_khz:	Current system clock frequency in kHz.
 */
unsigned int RCC_GetSysclkKhz(void) {
	return rcc_sysclk_khz;
}

/* CONFIGURE AND USE HSI AS SYSTEM CLOCK (16MHz INTERNAL RC).
 * @param:					None.
 * @return sysclk_on_hsi:	'1' if SYSCLK source was successfully switched to HSI, 0 otherwise.
 */
unsigned char RCC_SwitchToHsi(void) {
	// Init HSI.
	RCC -> CR |= (0b1 << 0); // Enable HSI (HSI16ON='1').
	// Wait for HSI to be stable.
	unsigned char sysclk_on_hsi = 0;
	unsigned int count = 0;
	while ((((RCC -> CR) & (0b1 << 2)) == 0) && (count < RCC_TIMEOUT_COUNT)) {
		RCC_Delay();
		count++; // Wait for HSIRDYF='1' or timeout.
	}
	// Check timeout.
	if (count < RCC_TIMEOUT_COUNT) {
		// Switch SYSCLK.
		RCC -> CFGR &= ~(0b11 << 0); // Reset bits 0-1.
		RCC -> CFGR |= (0b01 << 0); // Use HSI as system clock (SW='01').
		// Wait for clock switch.
		count = 0;
		while ((((RCC -> CFGR) & (0b11 << 2)) != (0b01 << 2)) && (count < RCC_TIMEOUT_COUNT)) {
			RCC_Delay();
			count++; // Wait for SWS='01' or timeout.
		}
		// Check timeout.
		if (count < RCC_TIMEOUT_COUNT) {
			// Disable MSI and HSE.
			RCC -> CR &= ~(0b1 << 8); // Disable MSI (MSION='0').
			RCC -> CR &= ~(0b1 << 16); // Disable HSE (HSEON='0').
			// Disable 16MHz TCXO power supply.
			GPIO_Write(&GPIO_TCXO16_POWER_ENABLE, 0);
			// Update flag and frequency.
			sysclk_on_hsi = 1;
			rcc_sysclk_khz = 16000;
		}
	}
	return sysclk_on_hsi;
}

/* CONFIGURE AND USE MSI AS SYSTEM CLOCK.
 * @param:					None.
 * @return sysclk_on_hsi:	'1' if SYSCLK source was successfully switched to MSI, 0 otherwise.
 */
unsigned char RCC_SwitchToMsi(void) {
	// Init MSI.
	RCC -> CR |= (0b1 << 8); // Enable MSI (MSION='1').
	RCC -> ICSCR &= ~(0b111 << 13); // Set frequency to 65kHz (MSIRANGE='000').
	// Wait for MSI to be stable.
	unsigned char sysclk_on_msi = 0;
	unsigned int count = 0;
	while ((((RCC -> CR) & (0b1 << 9)) == 0) && (count < RCC_TIMEOUT_COUNT)) {
		RCC_Delay();
		count++; // Wait for MSIRDYF='1' or timeout.
	}
	// Check timeout.
	if (count < RCC_TIMEOUT_COUNT) {
		// Switch SYSCLK.
		RCC -> CFGR &= ~(0b11 << 0); // Use MSI as system clock (SW='00').
		// Wait for clock switch.
		count = 0;
		while ((((RCC -> CFGR) & (0b11 << 2)) != (0b00 << 2)) && (count < RCC_TIMEOUT_COUNT)) {
			RCC_Delay();
			count++; // Wait for SWS='00' or timeout.
		}
		// Check timeout.
		if (count < RCC_TIMEOUT_COUNT) {
			// Disable HSI and HSE.
			RCC -> CR &= ~(0b1 << 0); // Disable HSI (HSI16ON='0').
			RCC -> CR &= ~(0b1 << 16); // Disable HSE (HSEON='0').
			// Disable 16MHz TCXO power supply.
			GPIO_Write(&GPIO_TCXO16_POWER_ENABLE, 0);
			// Update flag and frequency.
			sysclk_on_msi = 1;
			rcc_sysclk_khz = 65;
		}
	}
	return sysclk_on_msi;
}

/* CONFIGURE AND USE HSE AS SYSTEM CLOCK (16MHz TCXO).
 * @param:					None.
 * @return sysclk_on_hse:	'1' if SYSCLK source was successfully switched to HSE (TCXO), 0 otherwise.
 */
unsigned char RCC_SwitchToHse(void) {
	// Enable 16MHz TCXO.
	GPIO_Write(&GPIO_TCXO16_POWER_ENABLE, 1);
	// Init HSE.
	RCC -> CR |= (0b1 << 18); // Bypass oscillator (HSEBYP='1').
	RCC -> CR |= (0b1 << 16); // Enable HSE (HSEON='1').
	// Wait for HSE to be stable.
	unsigned char sysclk_on_hse = 0;
	unsigned int count = 0;
	while ((((RCC -> CR) & (0b1 << 17)) == 0) && (count < RCC_TIMEOUT_COUNT)) {
		RCC_Delay();
		count++; // Wait for HSERDY='1' or timeout.
	}
	// Check timeout.
	if (count < RCC_TIMEOUT_COUNT) {
		// Switch SYSCLK.
		RCC -> CFGR &= ~(0b11 << 0); // Reset bits 0-1.
		RCC -> CFGR |= (0b10 << 0); // Use HSE as system clock (SW='10').
		// Wait for clock switch.
		count = 0;
		while ((((RCC -> CFGR) & (0b11 << 2)) != (0b10 << 2)) && (count < RCC_TIMEOUT_COUNT)) {
			RCC_Delay();
			count++; // Wait for SWS='10' or timeout.
		}
		// Check timeout.
		if (count < RCC_TIMEOUT_COUNT) {
			// Disable MSI and HSI */
			RCC -> CR &= ~(0b1 << 8); // Disable MSI (MSION='0').
			RCC -> CR &= ~(0b1 << 0); // Disable HSI (HSI16ON='0').
			// Update flag and frequency.
			sysclk_on_hse = 1;
			rcc_sysclk_khz = 16000;
		}
	}
	// Switch TCXO and HSE off if any failure occured.
	if (sysclk_on_hse == 0) {
		GPIO_Write(&GPIO_TCXO16_POWER_ENABLE, 0);
		RCC -> CR &= ~(0b1 << 16); // Disable HSE (HSEON='0').
	}
	return sysclk_on_hse;
}

/* CONFIGURE AND USE LSI AS LOW SPEED OSCILLATOR (32kHz INTERNAL RC).
 * @param:					None.
 * @return lsi_available:	'1' if LSI was successfully started, 0 otherwise.
 */
unsigned char RCC_EnableLsi(void) {
	// Enable LSI.
	RCC -> CSR |= (0b1 << 0); // LSION='1'.
	// Wait for LSI to be stable.
	unsigned char lsi_available = 0;
	unsigned int count = 0;
	while ((((RCC -> CSR) & (0b1 << 1)) == 0) && (count < RCC_TIMEOUT_COUNT)) {
		RCC_Delay();
		count++; // Wait for LSIRDY='1'.
	}
	// Check timeout.
	if (count < RCC_TIMEOUT_COUNT) {
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
	// Local variables.
	unsigned int lsi_freq_hz = 0;
	// Init and start timers.
	LPTIM1_Init(LPTIM_MODE_LSI_CALIBRATION);
	TIM21_Init();
	LPTIM1_Start();
	TIM21_Start();
	// Wait 1 second.
	while (((TIM21 -> SR) & (0b1 << 0)) == 0); // Wait for first overflow.
	TIM21 -> SR &= ~(0b1 << 0); // Clear flag (UIF='0').
	// Get LSI frequency and stop timers.
	lsi_freq_hz = (LPTIM1 -> CNT);
	LPTIM1_Stop();
	TIM21_Stop();
	// Check value.
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
	// Configure drive level.
	//RCC -> CSR |= (0b11 << 11);
	// Enable LSE (32.768kHz crystal).
	RCC -> CSR |= (0b1 << 8); // LSEON='1'.
	// Wait for LSE to be stable.
	unsigned char lse_available = 0;
	unsigned int count = 0;
	while ((((RCC -> CSR) & (0b1 << 9)) == 0) && (count < RCC_TIMEOUT_COUNT)) {
		RCC_Delay();
		count++; // Wait for LSERDY='1'.
	}
	// Check timeout.
	if (count < RCC_TIMEOUT_COUNT) {
		// Update flag.
		lse_available = 1;
	}
	else {
		// Switch LSE off if any failure occured.
		RCC -> CSR &= ~(0b1 << 8); // LSEON='0'.
	}
	return lse_available;
}
