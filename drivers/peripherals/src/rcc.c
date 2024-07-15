/*
 * rcc.c
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#include "rcc.h"

#include "error.h"
#include "flash.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc_reg.h"
#include "tim.h"
#include "types.h"

/*** RCC local macros ***/

#define RCC_TIMEOUT_COUNT				1000000

#define RCC_LSI_FREQUENCY_DEFAULT_HZ	38000
#define RCC_LSI_FREQUENCY_MIN_HZ		26000
#define RCC_LSI_FREQUENCY_MAX_HZ		56000

#define RCC_LSE_FREQUENCY_HZ			32768

#define RCC_HSI_FREQUENCY_DEFAULT_HZ	16000000
#define RCC_HSI_FREQUENCY_MIN_HZ		15040000
#define RCC_HSI_FREQUENCY_MAX_HZ		16960000

#define RCC_HSE_FREQUENCY_HZ			16000000

/*** RCC local structures ***/

/*******************************************************************/
typedef struct {
	RCC_clock_t sysclk_source;
	uint32_t clock_frequency[RCC_CLOCK_LAST];
} RCC_context_t;

/*** RCC local global variables ***/

static const uint32_t RCC_MSI_CLOCK_FREQUENCY[RCC_MSI_RANGE_LAST] = {65536, 131072, 262144, 524288, 1048000, 2097000, 4194000};
static RCC_context_t rcc_ctx;

/*** RCC local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) RCC_IRQHandler(void) {
	// Clear flag.
	RCC -> CICR |= (0b1 << 0);
}

/*******************************************************************/
void _RCC_enable_lsi(void) {
	// Enable LSI.
	RCC -> CSR |= (0b1 << 0); // LSION='1'.
	// Enable interrupt.
	RCC -> CIER |= (0b1 << 0);
	NVIC_enable_interrupt(NVIC_INTERRUPT_RCC_CRS, NVIC_PRIORITY_RCC_CRS);
	// Wait for LSI to be stable.
	while (((RCC -> CSR) & (0b1 << 1)) == 0) {
		PWR_enter_sleep_mode();
	}
	NVIC_disable_interrupt(NVIC_INTERRUPT_RCC_CRS);
}

/*******************************************************************/
RCC_status_t _RCC_enable_lse(void) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	uint32_t loop_count = 0;
	// Enable LSE (32.768kHz crystal).
	RCC -> CSR |= (0b1 << 8); // LSEON='1'.
	// Wait for LSE to be stable.
	while (((RCC -> CSR) & (0b1 << 9)) == 0) {
		// Wait for LSERDY='1' ready flag or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			// Switch LSE off.
			RCC -> CSR &= ~(0b1 << 8); // LSEON='0'.
			// Exit loop.
			status = RCC_ERROR_LSE_READY;
			goto errors;
		}
	}
errors:
	return status;
}

/*** RCC functions ***/

/*******************************************************************/
RCC_status_t __attribute__((optimize("-O0"))) RCC_init(void) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	uint8_t i = 0;
	// Init context.
	rcc_ctx.clock_frequency[RCC_CLOCK_LSI] = RCC_LSI_FREQUENCY_DEFAULT_HZ;
	rcc_ctx.clock_frequency[RCC_CLOCK_LSE] = RCC_LSE_FREQUENCY_HZ;
	rcc_ctx.clock_frequency[RCC_CLOCK_MSI] = RCC_MSI_CLOCK_FREQUENCY[RCC_MSI_RANGE_5_2MHZ];
	rcc_ctx.clock_frequency[RCC_CLOCK_HSI] = RCC_HSI_FREQUENCY_DEFAULT_HZ;
	rcc_ctx.clock_frequency[RCC_CLOCK_HSE] = RCC_HSE_FREQUENCY_HZ;
	// Update system clock.
	rcc_ctx.sysclk_source = RCC_CLOCK_MSI;
	rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.sysclk_source];
	// Reset backup domain.
	RCC -> CSR |= (0b1 << 19); // RTCRST='1'.
	for (i=0 ; i<100 ; i++);
	RCC -> CSR &= ~(0b1 << 19); // RTCRST='0'.
	// Start low speed oscillators.
	_RCC_enable_lsi();
	status = _RCC_enable_lse();
	return status;
}

/*******************************************************************/
RCC_status_t RCC_switch_to_hsi(void) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	FLASH_status_t flash_status = FLASH_SUCCESS;
	uint32_t reg_cfgr = 0;
	uint32_t loop_count = 0;
	// Set flash latency.
	flash_status = FLASH_set_latency(1);
	FLASH_exit_error(RCC_ERROR_BASE_FLASH);
	// Enable HSI.
	RCC -> CR |= (0b1 << 0); // Enable HSI (HSI16ON='1').
	// Wait for HSI to be stable.
	while (((RCC -> CR) & (0b1 << 2)) == 0) {
		// Wait for HSIRDYF='1' or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_HSI_READY;
			goto errors;
		}
	}
	// Switch SYSCLK.
	reg_cfgr = (RCC -> CFGR);
	reg_cfgr &= ~(0b11 << 0); // Reset bits 0-1.
	reg_cfgr |= (0b01 << 0); // Use HSI as system clock (SW='01').
	RCC -> CFGR = reg_cfgr;
	// Wait for clock switch.
	loop_count = 0;
	while (((RCC -> CFGR) & (0b11 << 2)) != (0b01 << 2)) {
		// Wait for SWS='01' or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_HSI_SWITCH;
			goto errors;
		}
	}
	// Update clocks context.
	rcc_ctx.sysclk_source = RCC_CLOCK_HSI;
errors:
	// Update system clock.
	rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.sysclk_source];
	return status;
}

/*******************************************************************/
RCC_status_t RCC_switch_to_msi(RCC_msi_range_t msi_range) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	FLASH_status_t flash_status = FLASH_SUCCESS;
	uint32_t loop_count = 0;
	// Check parameter.
	if (msi_range >= RCC_MSI_RANGE_LAST) {
		status = RCC_ERROR_MSI_RANGE;
		goto errors;
	}
	// Set frequency.
	RCC -> ICSCR &= ~(0b111 << 13);
	RCC -> ICSCR |= (msi_range << 13);
	// Enable MSI.
	RCC -> CR |= (0b1 << 8); // MSION='1'.
	// Wait for MSI to be stable.
	while (((RCC -> CR) & (0b1 << 9)) == 0) {
		// Wait for MSIRDYF='1' or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_MSI_READY;
			goto errors;
		}
	}
	// Switch SYSCLK.
	RCC -> CFGR &= ~(0b11 << 0); // Use MSI as system clock (SW='00').
	// Wait for clock switch.
	while (((RCC -> CFGR) & (0b11 << 2)) != (0b00 << 2)) {
		// Wait for SWS='00' or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_MSI_SWITCH;
			goto errors;
		}
	}
	// Set flash latency.
	flash_status = FLASH_set_latency(0);
	FLASH_exit_error(RCC_ERROR_BASE_FLASH);
	// Update clocks context.
	rcc_ctx.sysclk_source = RCC_CLOCK_MSI;
	rcc_ctx.clock_frequency[RCC_CLOCK_MSI] = RCC_MSI_CLOCK_FREQUENCY[msi_range];
errors:
	// Update system clock.
	rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.sysclk_source];
	return status;
}

/*******************************************************************/
RCC_status_t RCC_calibrate(void) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	TIM_status_t tim21_status = TIM_SUCCESS;
	uint16_t ref_clock_pulse_count = 0;
	uint16_t mco_pulse_count = 0;
	uint64_t temp_u64 = 0;
	uint32_t clock_frequency_hz = 0;
	uint8_t lse_status = 0;
	// Init measurement timer.
	TIM21_init();
	// Check LSE status.
	status = RCC_get_status(RCC_CLOCK_LSE, &lse_status);
	if (status != RCC_SUCCESS) goto errors;
	// HSI calibration is not possible without LSE.
	if (lse_status == 0) goto lsi_calibration;
	// Connect MCO to LSE clock.
	RCC -> CFGR &= ~(0x7F << 24);
	RCC -> CFGR |= (0b0111 << 24);
	// Perform measurement.
	tim21_status = TIM21_mco_capture(&ref_clock_pulse_count, &mco_pulse_count);
	TIM21_stack_error();
	// Compute HSI frequency.
	temp_u64 = ((uint64_t) RCC_LSE_FREQUENCY_HZ * (uint64_t) ref_clock_pulse_count);
	clock_frequency_hz = (uint32_t) ((temp_u64) / ((uint64_t) mco_pulse_count));
	// Check value.
	if ((tim21_status != TIM_SUCCESS) || (clock_frequency_hz < RCC_HSI_FREQUENCY_MIN_HZ) || (clock_frequency_hz > RCC_HSI_FREQUENCY_MAX_HZ)) {
		// Set to default value if out of expected range
		clock_frequency_hz = RCC_HSI_FREQUENCY_DEFAULT_HZ;
		ERROR_stack_add(ERROR_BASE_RCC + RCC_ERROR_HSI_CALIBRATION);
	}
	// Update local data.
	rcc_ctx.clock_frequency[RCC_CLOCK_HSI] = clock_frequency_hz;
lsi_calibration:
	// Connect MCO to LSI clock.
	RCC -> CFGR &= ~(0x7F << 24);
	RCC -> CFGR |= (0b0110 << 24);
	// Perform measurement.
	tim21_status = TIM21_mco_capture(&ref_clock_pulse_count, &mco_pulse_count);
	TIM21_stack_error();
	// Compute LSI frequency.
	temp_u64 = ((uint64_t) rcc_ctx.clock_frequency[RCC_CLOCK_HSI] * (uint64_t) mco_pulse_count);
	clock_frequency_hz = (uint32_t) ((temp_u64) / ((uint64_t) ref_clock_pulse_count));
	// Check value.
	if ((tim21_status != TIM_SUCCESS) || (clock_frequency_hz < RCC_LSI_FREQUENCY_MIN_HZ) || (clock_frequency_hz > RCC_LSI_FREQUENCY_MAX_HZ)) {
		// Set to default value if out of expected range
		clock_frequency_hz = RCC_LSI_FREQUENCY_DEFAULT_HZ;
		ERROR_stack_add(ERROR_BASE_RCC + RCC_ERROR_LSI_CALIBRATION);
	}
	// Update local data.
	rcc_ctx.clock_frequency[RCC_CLOCK_LSI] = clock_frequency_hz;
errors:
	// Release timer.
	TIM21_de_init();
	// Update system clock.
	rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.sysclk_source];
	return status;
}

/*******************************************************************/
RCC_status_t RCC_get_frequency_hz(RCC_clock_t clock, uint32_t* frequency_hz) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	// Check parameters.
	if (clock >= RCC_CLOCK_LAST) {
		status = RCC_ERROR_CLOCK;
		goto errors;
	}
	if (frequency_hz == NULL) {
		status = RCC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Read frequency.
	(*frequency_hz) = rcc_ctx.clock_frequency[clock];
errors:
	return status;
}

/*******************************************************************/
RCC_status_t RCC_get_status(RCC_clock_t clock, uint8_t* clock_is_ready) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	// Check parameters.
	if (clock_is_ready == NULL) {
		status = RCC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Check clock.
	switch (clock) {
	case RCC_CLOCK_LSI:
		(*clock_is_ready) = (((RCC -> CSR) >> 1) & 0b1);
		break;
	case RCC_CLOCK_LSE:
		(*clock_is_ready) = (((RCC -> CSR) >> 9) & 0b1);
		break;
	case RCC_CLOCK_MSI:
		(*clock_is_ready) = (((RCC -> CR) >> 9) & 0b1);
		break;
	case RCC_CLOCK_HSI:
		(*clock_is_ready) = (((RCC -> CR) >> 2) & 0b1);
		break;
	case RCC_CLOCK_HSE:
		(*clock_is_ready) = (((RCC -> CR) >> 17) & 0b1);
		break;
	case RCC_CLOCK_SYSTEM:
		(*clock_is_ready) = 1;
		break;
	default:
		status = RCC_ERROR_CLOCK;
		goto errors;
	}
errors:
	return status;
}
