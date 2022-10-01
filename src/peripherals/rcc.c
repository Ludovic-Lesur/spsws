/*
 * rcc.c
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#include "rcc.h"

#include "flash.h"
#include "gpio.h"
#include "mapping.h"
#include "rcc_reg.h"
#include "tim.h"

/*** RCC local macros ***/

#define RCC_TIMEOUT_COUNT				10
#define RCC_MSI_RESET_FREQUENCY_KHZ		2100

#define RCC_LSI_AVERAGING_COUNT			5
#define RCC_LSI_FREQUENCY_MIN_HZ		26000
#define RCC_LSI_FREQUENCY_MAX_HZ		56000

/*** RCC local global variables ***/

static unsigned int rcc_sysclk_khz;

/*** RCC local functions ***/

/* PERFORM A MANUAL DELAY.
 * @param:	None.
 * @return:	None.
 */
static void RCC_delay(void) {
	// Local variables.
	unsigned int j = 0;
	unsigned int loop_count = (19 * rcc_sysclk_khz) / 3; // Value for 100ms.
	// Perform delay.
	for (j=0 ; j<loop_count ; j++) {
		// Poll a bit always read as '0' (required to avoid compiler optimization).
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
void RCC_init(void) {
	// Default prescalers (HCLK, PCLK1 and PCLK2 must not exceed 32MHz).
	// HCLK = SYSCLK = 16MHz (HPRE='0000').
	// PCLK1 = HCLK = 16MHz (PPRE1='000').
	// PCLK2 = HCLK = 16MHz (PPRE2='000').
	// All peripherals clocked via the corresponding APBx line.
	// Configure TCXO power control pin.
	GPIO_configure(&GPIO_TCXO16_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_write(&GPIO_TCXO16_POWER_ENABLE, 0);
	// Reset clock is MSI 2.1MHz.
	rcc_sysclk_khz = RCC_MSI_RESET_FREQUENCY_KHZ;
}

/* CONFIGURE AND USE HSI AS SYSTEM CLOCK (16MHz INTERNAL RC).
 * @param:			None.
 * @return status:	Function execution status.
 */
RCC_status_t RCC_switch_to_hsi(void) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	FLASH_status_t flash_status = FLASH_SUCCESS;
	unsigned int loop_count = 0;
	// Set flash latency.
	flash_status = FLASH_set_latency(1);
	FLASH_status_check(RCC_ERROR_BASE_FLASH);
	// Init HSI.
	RCC -> CR |= (0b1 << 0); // Enable HSI (HSI16ON='1').
	// Wait for HSI to be stable.
	while (((RCC -> CR) & (0b1 << 2)) == 0) {
		// Wait for HSIRDYF='1' or timeout.
		RCC_delay();
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_HSI_READY;
			goto errors;
		}
	}
	// Switch SYSCLK.
	RCC -> CFGR &= ~(0b11 << 0); // Reset bits 0-1.
	RCC -> CFGR |= (0b01 << 0); // Use HSI as system clock (SW='01').
	// Wait for clock switch.
	loop_count = 0;
	while (((RCC -> CFGR) & (0b11 << 2)) != (0b01 << 2)) {
		// Wait for SWS='01' or timeout.
		RCC_delay();
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_HSI_SWITCH;
			goto errors;
		}
	}
	// Disable MSI and HSE.
	RCC -> CR &= ~(0b1 << 8); // Disable MSI (MSION='0').
	RCC -> CR &= ~(0b1 << 16); // Disable HSE (HSEON='0').
	// Disable 16MHz TCXO power supply.
	GPIO_write(&GPIO_TCXO16_POWER_ENABLE, 0);
	// Update flag and frequency.
	rcc_sysclk_khz = RCC_HSI_FREQUENCY_KHZ;
errors:
	return status;
}

/* CONFIGURE AND USE HSE AS SYSTEM CLOCK (16MHz TCXO).
 * @param:			None.
 * @return status:	Function execution status.
 */
RCC_status_t RCC_switch_to_hse(void) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	FLASH_status_t flash_status = FLASH_SUCCESS;
	unsigned int loop_count = 0;
	// Set flash latency.
	flash_status = FLASH_set_latency(1);
	FLASH_status_check(RCC_ERROR_BASE_FLASH);
	// Enable 16MHz TCXO.
	GPIO_write(&GPIO_TCXO16_POWER_ENABLE, 1);
	// Init HSE.
	RCC -> CR |= (0b1 << 18); // Bypass oscillator (HSEBYP='1').
	RCC -> CR |= (0b1 << 16); // Enable HSE (HSEON='1').
	// Wait for HSE to be stable.
	while (((RCC -> CR) & (0b1 << 17)) == 0) {
		// Wait for HSERDY='1' or timeout.
		RCC_delay();
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_HSE_READY;
			// Turn TCXO off.
			GPIO_write(&GPIO_TCXO16_POWER_ENABLE, 0);
			RCC -> CR &= ~(0b1 << 16); // Disable HSE (HSEON='0').
			goto errors;
		}
	}
	// Switch SYSCLK.
	RCC -> CFGR &= ~(0b11 << 0); // Reset bits 0-1.
	RCC -> CFGR |= (0b10 << 0); // Use HSE as system clock (SW='10').
	// Wait for clock switch.
	loop_count = 0;
	while (((RCC -> CFGR) & (0b11 << 2)) != (0b10 << 2)) {
		// Wait for SWS='10' or timeout.
		RCC_delay();
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_HSE_SWITCH;
			// Turn TCXO off.
			GPIO_write(&GPIO_TCXO16_POWER_ENABLE, 0);
			RCC -> CR &= ~(0b1 << 16); // Disable HSE (HSEON='0').
			goto errors;
		}
	}
	// Disable MSI and HSI.
	RCC -> CR &= ~(0b1 << 8); // Disable MSI (MSION='0').
	RCC -> CR &= ~(0b1 << 0); // Disable HSI (HSI16ON='0').
	// Update flag and frequency.
	rcc_sysclk_khz = RCC_TCXO_FREQUENCY_KHZ;
errors:
	return status;
}

/* RETURN THE CURRENT SYSTEM CLOCK FREQUENCY.
 * @param:					None.
 * @return rcc_sysclk_khz:	Current system clock frequency in kHz.
 */
unsigned int RCC_get_sysclk_khz(void) {
	return rcc_sysclk_khz;
}

/* CONFIGURE AND USE LSI AS LOW SPEED OSCILLATOR (32kHz INTERNAL RC).
 * @param:			None.
 * @return status:	Function execution status.
 */
RCC_status_t RCC_enable_lsi(void) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	unsigned int loop_count = 0;
	// Enable LSI.
	RCC -> CSR |= (0b1 << 0); // LSION='1'.
	// Wait for LSI to be stable.
	while (((RCC -> CSR) & (0b1 << 1)) == 0) {
		// Wait for LSIRDY='1' or timeout.
		RCC_delay();
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_LSI_READY;
			break;
		}
	}
	return status;
}

/* COMPUTE EFFECTIVE LSI OSCILLATOR FREQUENCY.
 * @param lsi_frequency_hz:		Pointer that will contain measured LSI frequency in Hz.
 * @return status:				Function execution status.
 */
RCC_status_t RCC_get_lsi_frequency(unsigned int* lsi_frequency_hz) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	TIM_status_t tim21_status = TIM_SUCCESS;
	unsigned int lsi_frequency_sample = 0;
	unsigned char sample_idx = 0;
	// Reset result.
	(*lsi_frequency_hz) = 0;
	// Init measurement timer.
	TIM21_init();
	// Compute average.
	for (sample_idx=0 ; sample_idx<RCC_LSI_AVERAGING_COUNT ; sample_idx++) {
		// Perform measurement.
		tim21_status = TIM21_get_lsi_frequency(&lsi_frequency_sample);
		TIM21_status_check(RCC_ERROR_BASE_TIM);
		(*lsi_frequency_hz) = (((*lsi_frequency_hz) * sample_idx) + lsi_frequency_sample) / (sample_idx + 1);
	}
	// Check value.
	if (((*lsi_frequency_hz) < RCC_LSI_FREQUENCY_MIN_HZ) || ((*lsi_frequency_hz) > RCC_LSI_FREQUENCY_MAX_HZ)) {
		// Set to default value if out of expected range
		(*lsi_frequency_hz) = RCC_LSI_FREQUENCY_HZ;
		status = RCC_ERROR_LSI_MEASUREMENT;
	}
errors:
	TIM21_disable();
	return status;
}

/* ENABLE LSE OSCILLATOR (32kHz EXTERNAL QUARTZ).
 * @param:			None.
 * @return status:	Function execution status.
 */
RCC_status_t RCC_enable_lse(void) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	unsigned int loop_count = 0;
	// Configure drive level.
	//RCC -> CSR |= (0b11 << 11);
	// Enable LSE (32.768kHz crystal).
	RCC -> CSR |= (0b1 << 8); // LSEON='1'.
	// Wait for LSE to be stable.
	while (((RCC -> CSR) & (0b1 << 9)) == 0) {
		RCC_delay();
		loop_count++; // Wait for LSERDY='1'.
		if (loop_count > RCC_TIMEOUT_COUNT) {
			// Turn LSE off.
			RCC -> CSR &= ~(0b1 << 8);
			status = RCC_ERROR_LSE_READY;
			break;
		}
	}
	return status;
}
