/*!*****************************************************************
 * \file    rf_api.c
 * \brief   Radio drivers.
 *******************************************************************
 * \copyright
 *
 * Copyright (c) 2022, UnaBiz SAS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1 Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  2 Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************/

#include "manuf/rf_api.h"

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"

#include "error.h"
#include "error_base.h"
#include "exti.h"
#include "gpio_mapping.h"
#include "iwdg.h"
#include "manuf/mcu_api.h"
#include "nvic_priority.h"
#include "power.h"
#include "pwr.h"
#include "rfe.h"
#include "sx1232.h"
#include "types.h"

/*** RF API local macros ***/

#define RF_API_MODULATION_TIMER_INSTANCE	TIM_INSTANCE_TIM22

#define RF_API_SYMBOL_PROFILE_SIZE_BYTES	40

/*** RF API local structures ***/

/*******************************************************************/
typedef enum {
	// Driver errors.
	RF_API_ERROR_NULL_PARAMETER = (RF_API_SUCCESS + 1),
	RF_API_ERROR_BUFFER_SIZE,
	RF_API_ERROR_STATE,
	RF_API_ERROR_MODULATION,
	RF_API_ERROR_MODE,
	RF_API_ERROR_LATENCY_TYPE,
	// Low level drivers errors.
	RF_API_ERROR_DRIVER_TIMER_MODULATION,
	RF_API_ERROR_DRIVER_MCU_API,
	RF_API_ERROR_DRIVER_POWER,
	RF_API_ERROR_DRIVER_SX1232,
	RF_API_ERROR_DRIVER_RFE
} RF_API_custom_status_t;

/*******************************************************************/
typedef enum {
	RF_API_STATE_READY = 0,
	RF_API_STATE_TX_START,
	RF_API_STATE_TX_RAMP_UP,
	RF_API_STATE_TX_BITSTREAM,
	RF_API_STATE_TX_RAMP_DOWN,
	RF_API_STATE_TX_END,
#ifdef BIDIRECTIONAL
	RF_API_STATE_RX_START,
	RF_API_STATE_RX,
#endif
	RF_API_STATE_LAST
} RF_API_state_t;

/*******************************************************************/
typedef union {
	struct {
		unsigned timer_irq_enable : 1;
		unsigned timer_irq_flag : 1;
		unsigned gpio_irq_enable : 1;
		unsigned gpio_irq_flag : 1;
	} field;
	sfx_u8 all;
} RF_API_flags_t;

/*******************************************************************/
typedef struct {
	// Common.
	RF_API_state_t state;
	volatile RF_API_flags_t flags;
	// TX.
	sfx_u8 tx_bitstream[SIGFOX_UL_BITSTREAM_SIZE_BYTES];
	sfx_u8 tx_bitstream_size_bytes;
	sfx_u8 tx_byte_idx;
	sfx_u8 tx_bit_idx;
	sfx_u8 tx_symbol_profile_idx;
#ifdef BIDIRECTIONAL
	// RX.
	sfx_u8 dl_phy_content[SIGFOX_DL_PHY_CONTENT_SIZE_BYTES];
	sfx_u8 dl_rssi_read_flag;
	sfx_s16 dl_rssi_dbm;
#endif
} RF_API_context_t;

/*** RF API local global variables ***/

static const sfx_u8 RF_API_RAMP_AMPLITUDE_PROFILE[RF_API_SYMBOL_PROFILE_SIZE_BYTES] =
	{0, 8, 16, 23, 29, 36, 43, 49, 55, 60, 65, 70, 74, 78, 81, 84, 86, 88, 89, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};
static const sfx_u8 RF_API_BIT0_AMPLITUDE_PROFILE[RF_API_SYMBOL_PROFILE_SIZE_BYTES] =
	{90, 89, 88, 86, 84, 81, 78, 74, 70, 65, 60, 55, 49, 43, 36, 29, 23, 16, 8, 0, 0, 8, 16, 23, 29, 36, 43, 49, 55, 60, 65, 70, 74, 78, 81, 84, 86, 88, 89, 90};
#ifdef BIDIRECTIONAL
static const sfx_u8 RF_API_DL_FT[SIGFOX_DL_FT_SIZE_BYTES] = SIGFOX_DL_FT;
#endif
#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
static sfx_u32 RF_API_LATENCY_MS[RF_API_LATENCY_LAST] = {
	POWER_ON_DELAY_MS_RADIO_TCXO, // Wake-up.
	(POWER_ON_DELAY_MS_RADIO + SX1232_OSCILLATOR_DELAY_MS + 1), // TX init (power on delay + 750us).
	0, // Send start (depends on bit rate and will be computed during init function).
	0, // Send stop (depends on bit rate and will be computed during init function).
	0, // TX de-init (215us).
	0, // Sleep.
#ifdef BIDIRECTIONAL
	(POWER_ON_DELAY_MS_RADIO + SX1232_OSCILLATOR_DELAY_MS + 2), // RX init (power on delay + 1.06ms).
	SX1232_START_RX_DELAY_MS, // Receive start.
	10, // Receive stop (10ms).
	0, // RX de-init (215us).
#endif
};
#endif
static RF_API_context_t rf_api_ctx;

/*** RF API local functions ***/

/*******************************************************************/
static void _RF_API_modulation_timer_irq_callback(void) {
	// Set flag.
	rf_api_ctx.flags.field.timer_irq_flag = rf_api_ctx.flags.field.timer_irq_enable;
}

#ifdef BIDIRECTIONAL
/*******************************************************************/
static void _RF_API_sx1232_gpio_irq_callback(void) {
	// Set flag.
	rf_api_ctx.flags.field.gpio_irq_flag = rf_api_ctx.flags.field.gpio_irq_enable;
}
#endif

/*******************************************************************/
static RF_API_status_t _RF_API_internal_process(void) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	sfx_u8 symbol_profile_idx = 0;
#ifdef BIDIRECTIONAL
	RFE_status_t rfe_status = RFE_SUCCESS;
#endif
	// Perform state machine.
	switch (rf_api_ctx.state) {
	case RF_API_STATE_READY:
		// Nothing to do.
		break;
	case RF_API_STATE_TX_START:
		// Start radio.
		SX1232_set_pa_power_value(0x00);
		sx1232_status = SX1232_start_tx();
		SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
		// Enable interrupt.
		rf_api_ctx.flags.field.timer_irq_enable = 1;
		// Update state.
		rf_api_ctx.state = RF_API_STATE_TX_RAMP_UP;
		break;
	case RF_API_STATE_TX_RAMP_UP:
		// Load ramp profile.
		SX1232_set_pa_power_value(RF_API_RAMP_AMPLITUDE_PROFILE[rf_api_ctx.tx_symbol_profile_idx]);
		// Increment symbol profile index.
		rf_api_ctx.tx_symbol_profile_idx++;
		if (rf_api_ctx.tx_symbol_profile_idx >= RF_API_SYMBOL_PROFILE_SIZE_BYTES) {
			rf_api_ctx.tx_symbol_profile_idx = 0;
			// Update state.
			rf_api_ctx.state = RF_API_STATE_TX_BITSTREAM;
		}
		break;
	case RF_API_STATE_TX_BITSTREAM:
		// Check bit.
		if ((rf_api_ctx.tx_bitstream[rf_api_ctx.tx_byte_idx] & (1 << (7 - rf_api_ctx.tx_bit_idx))) == 0) {
			// Invert phase at the middle of the symbol profile.
			if (rf_api_ctx.tx_symbol_profile_idx == (RF_API_SYMBOL_PROFILE_SIZE_BYTES >> 1)) {
				GPIO_SX1232_DIO2_HIGH();
			}
			else {
				GPIO_SX1232_DIO2_LOW();
			}
			// Amplitude shaping.
			symbol_profile_idx = rf_api_ctx.tx_symbol_profile_idx;
		}
		else {
			// Constant CW.
			symbol_profile_idx = 0;
		}
		SX1232_set_pa_power_value(RF_API_BIT0_AMPLITUDE_PROFILE[symbol_profile_idx]);
		// Increment symbol profile index.
		rf_api_ctx.tx_symbol_profile_idx++;
		if (rf_api_ctx.tx_symbol_profile_idx >= RF_API_SYMBOL_PROFILE_SIZE_BYTES) {
			rf_api_ctx.tx_symbol_profile_idx = 0;
			// Increment bit index.
			rf_api_ctx.tx_bit_idx++;
			if (rf_api_ctx.tx_bit_idx >= 8) {
				rf_api_ctx.tx_bit_idx = 0;
				// Increment byte index.
				rf_api_ctx.tx_byte_idx++;
				// Check end of bitstream.
				if (rf_api_ctx.tx_byte_idx >= (rf_api_ctx.tx_bitstream_size_bytes)) {
					rf_api_ctx.tx_byte_idx = 0;
					// Update state.
					rf_api_ctx.state = RF_API_STATE_TX_RAMP_DOWN;
				}
			}
		}
		break;
	case RF_API_STATE_TX_RAMP_DOWN:
		// Load ramp profile.
		SX1232_set_pa_power_value(RF_API_RAMP_AMPLITUDE_PROFILE[RF_API_SYMBOL_PROFILE_SIZE_BYTES - rf_api_ctx.tx_symbol_profile_idx - 1]);
		// Increment symbol profile index.
		rf_api_ctx.tx_symbol_profile_idx++;
		if (rf_api_ctx.tx_symbol_profile_idx >= RF_API_SYMBOL_PROFILE_SIZE_BYTES) {
			rf_api_ctx.tx_symbol_profile_idx = 0;
			// Update state.
			rf_api_ctx.state = RF_API_STATE_TX_END;
		}
		break;
	case RF_API_STATE_TX_END:
		// Disable interrupt.
		rf_api_ctx.flags.field.timer_irq_enable = 0;
		// Stop radio.
		SX1232_set_pa_power_value(0x00);
		sx1232_status = SX1232_set_mode(SX1232_MODE_STANDBY);
		SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
		// Update state.
		rf_api_ctx.state = RF_API_STATE_READY;
		break;
#ifdef BIDIRECTIONAL
	case RF_API_STATE_RX_START:
		// Start radio.
		sx1232_status = SX1232_start_rx();
		SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
		// Enable interrupt.
		rf_api_ctx.flags.field.gpio_irq_enable = 1;
		// Update state.
		rf_api_ctx.state = RF_API_STATE_RX;
		break;
	case RF_API_STATE_RX:
		// Check flag.
		if (rf_api_ctx.flags.field.gpio_irq_flag != 0) {
			// Read RSSI.
			rfe_status = RFE_get_rssi(&rf_api_ctx.dl_rssi_dbm);
			RFE_stack_exit_error(ERROR_BASE_RFE, (RF_API_status_t) RF_API_ERROR_DRIVER_RFE);
			// Stop radio.
			sx1232_status = SX1232_set_mode(SX1232_MODE_STANDBY);
			SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
			// Read FIFO
			sx1232_status = SX1232_read_fifo((sfx_u8*) rf_api_ctx.dl_phy_content, SIGFOX_DL_PHY_CONTENT_SIZE_BYTES);
			SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
			// Disable interrupt.
			rf_api_ctx.flags.field.gpio_irq_enable = 0;
			// Update state.
			rf_api_ctx.state = RF_API_STATE_READY;
		}
		break;
#endif /* BIDIRECTIONAL */
	default:
		EXIT_ERROR((RF_API_status_t) RF_API_ERROR_STATE);
		break;
	}
errors:
	RETURN();
}

/*** RF API functions ***/

#if (defined ASYNCHRONOUS) || (defined LOW_LEVEL_OPEN_CLOSE)
/*******************************************************************/
RF_API_status_t RF_API_open(RF_API_config_t *rf_api_config) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	// Ignore unused parameters.
	UNUSED(rf_api_config);
	// Return.
	RETURN();
}
#endif

#ifdef LOW_LEVEL_OPEN_CLOSE
/*******************************************************************/
RF_API_status_t RF_API_close(void) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	RETURN();
}
#endif

#ifdef ASYNCHRONOUS
/*******************************************************************/
RF_API_status_t RF_API_process(void) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	RETURN();
}
#endif

/*******************************************************************/
RF_API_status_t RF_API_wake_up(void) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	POWER_status_t power_status = POWER_SUCCESS;
	// Turn radio TCXO on.
	power_status = POWER_enable(POWER_DOMAIN_RADIO_TCXO, LPTIM_DELAY_MODE_SLEEP);
	POWER_stack_exit_error(ERROR_BASE_POWER, (RF_API_status_t) RF_API_ERROR_DRIVER_POWER);
errors:
	RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_sleep(void) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	POWER_status_t power_status = POWER_SUCCESS;
	// Turn radio TCXO off.
	power_status = POWER_disable(POWER_DOMAIN_RADIO_TCXO);
	POWER_stack_exit_error(ERROR_BASE_POWER, (RF_API_status_t) RF_API_ERROR_DRIVER_POWER);
errors:
	RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_init(RF_API_radio_parameters_t *radio_parameters) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	POWER_status_t power_status = POWER_SUCCESS;
	TIM_status_t tim_status = TIM_SUCCESS;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	RFE_status_t rfe_status = RFE_SUCCESS;
	SX1232_modulation_t modulation = SX1232_MODULATION_LAST;
	SX1232_modulation_shaping_t modulation_shaping = SX1232_MODULATION_SHAPING_LAST;
	SX1232_data_mode_t data_mode = SX1232_DATA_MODE_LAST;
	sfx_u32 bitrate_bps = 0;
	sfx_u32 deviation_hz = 0;
	sfx_u32 frequency_hz = 0;
	sfx_u32 modulation_timer_period_ns = 0;
#ifdef PARAMETERS_CHECK
	// Check parameter.
	if (radio_parameters == SFX_NULL) {
		EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
	}
#endif
	// Turn radio on.
	power_status = POWER_enable(POWER_DOMAIN_RADIO, LPTIM_DELAY_MODE_SLEEP);
	POWER_stack_exit_error(ERROR_BASE_POWER, (RF_API_status_t) RF_API_ERROR_DRIVER_POWER);
	// Init transceiver.
	sx1232_status = SX1232_set_mode(SX1232_MODE_SLEEP);
	SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
	// Oscillator.
	sx1232_status = SX1232_set_oscillator(SX1232_OSCILLATOR_TCXO);
	SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
	// Modulation parameters.
	switch (radio_parameters -> modulation) {
	case RF_API_MODULATION_NONE:
		modulation = SX1232_MODULATION_OOK;
		modulation_shaping = SX1232_MODULATION_SHAPING_NONE;
		bitrate_bps = (radio_parameters -> bit_rate_bps);
#ifdef BIDIRECTIONAL
		deviation_hz = (radio_parameters -> deviation_hz);
#endif
		frequency_hz = (radio_parameters -> frequency_hz);
		data_mode = SX1232_DATA_MODE_CONTINUOUS;
		// Set CW output power.
		sx1232_status = SX1232_set_rf_output_power(radio_parameters -> tx_power_dbm_eirp);
		SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
		break;
	case RF_API_MODULATION_DBPSK:
		modulation = SX1232_MODULATION_FSK;
		modulation_shaping = SX1232_MODULATION_SHAPING_NONE;
		bitrate_bps = 0; // Unused directly.
		deviation_hz = ((radio_parameters -> bit_rate_bps) * RF_API_SYMBOL_PROFILE_SIZE_BYTES) / (4);
		frequency_hz = (radio_parameters -> frequency_hz) + deviation_hz;
		data_mode = SX1232_DATA_MODE_CONTINUOUS;
		// Use manual control of the PA.
		sx1232_status = SX1232_enable_manual_pa_control();
		SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
		// Init symbol profile timer.
		modulation_timer_period_ns = (MATH_POWER_10[9]) / ((radio_parameters -> bit_rate_bps) * RF_API_SYMBOL_PROFILE_SIZE_BYTES);
		tim_status = TIM_STD_init(RF_API_MODULATION_TIMER_INSTANCE, modulation_timer_period_ns, NVIC_PRIORITY_SIGFOX_MODULATION_TIMER, &_RF_API_modulation_timer_irq_callback);
		TIM_stack_exit_error(ERROR_BASE_TIM_MODULATION, (RF_API_status_t) RF_API_ERROR_DRIVER_TIMER_MODULATION);
		break;
	case RF_API_MODULATION_GFSK:
		modulation = SX1232_MODULATION_FSK;
		modulation_shaping = SX1232_MODULATION_SHAPING_FSK_BT_1;
		bitrate_bps = (radio_parameters -> bit_rate_bps);
#ifdef BIDIRECTIONAL
		deviation_hz = (radio_parameters -> deviation_hz);
#endif
		frequency_hz = (radio_parameters -> frequency_hz);
		data_mode = SX1232_DATA_MODE_PACKET;
		break;
	default:
		EXIT_ERROR((RF_API_status_t) RF_API_ERROR_MODULATION);
		break;
	}
	// Frequency.
	sx1232_status = SX1232_set_rf_frequency(frequency_hz);
	SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
	// Modulation.
	sx1232_status = SX1232_set_modulation(modulation, modulation_shaping);
	SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
	// Bit rate.
	if (bitrate_bps != 0) {
		sx1232_status = SX1232_set_bitrate(bitrate_bps);
		SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
	}
	// Deviation.
	if (deviation_hz != 0) {
		sx1232_status = SX1232_set_fsk_deviation(deviation_hz);
		SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
	}
	// Data mode.
	sx1232_status = SX1232_set_data_mode(data_mode);
	SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
	// Configure specific registers.
	switch (radio_parameters -> rf_mode) {
	case RF_API_MODE_TX:
		// Specific uplink settings.
		sx1232_status = SX1232_set_pll_mode(SX1232_PLL_MODE_LOW_PHASE_NOISE);
		SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
		// Start latency = ramp-up.
		RF_API_LATENCY_MS[RF_API_LATENCY_SEND_START] = SX1232_START_TX_DELAY_MS + ((1000) / ((sfx_u32) (radio_parameters -> bit_rate_bps)));
		// Stop latency = ramp-down.
		RF_API_LATENCY_MS[RF_API_LATENCY_SEND_STOP] = ((1000) / ((sfx_u32) (radio_parameters -> bit_rate_bps)));
#endif
		// Init DIO2 for bitstream transmission.
		GPIO_configure(&GPIO_SX1232_DIO2, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
		// Switch to TX.
		sx1232_status = SX1232_set_rf_output_pin(SX1232_RF_OUTPUT_PIN_PABOOST);
		SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
		rfe_status = RFE_set_path(RFE_PATH_TX_BYPASS);
		RFE_stack_exit_error(ERROR_BASE_RFE, (RF_API_status_t) RF_API_ERROR_DRIVER_RFE);
		break;
#ifdef BIDIRECTIONAL
	case RF_API_MODE_RX:
		// Specific downlink settings.
		sx1232_status = SX1232_set_rx_bandwidth(SX1232_RXBW_MANTISSA_24, SX1232_RXBW_EXPONENT_7);
		SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
		sx1232_status = SX1232_set_lna_configuration(SX1232_LNA_MODE_BOOST, SX1232_LNA_GAIN_ATTENUATION_0DB, 0);
		SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
		sx1232_status = SX1232_set_rssi_sampling(SX1232_RSSI_SAMPLING_256);
		SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
		sx1232_status = SX1232_set_preamble_detector(1, 0);
		SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
		sx1232_status = SX1232_set_sync_word((sfx_u8*) RF_API_DL_FT, SIGFOX_DL_FT_SIZE_BYTES);
		SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
		sx1232_status = SX1232_set_data_size(SIGFOX_DL_PHY_CONTENT_SIZE_BYTES);
		SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
		// Init DIO0 to detect payload ready interrupt.
		sx1232_status = SX1232_set_dio_mapping(SX1232_DIO0, SX1232_DIO_MAPPING0);
		SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
		GPIO_configure(&GPIO_SX1232_DIO0, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
		EXTI_configure_gpio(&GPIO_SX1232_DIO0, EXTI_TRIGGER_RISING_EDGE, &_RF_API_sx1232_gpio_irq_callback, NVIC_PRIORITY_SIGFOX_MODULATION_GPIO);
		// Switch to RX.
		rfe_status = RFE_set_path(RFE_PATH_RX_LNA);
		RFE_stack_exit_error(ERROR_BASE_RFE, (RF_API_status_t) RF_API_ERROR_DRIVER_RFE);
		break;
#endif
	default:
		EXIT_ERROR((RF_API_status_t) RF_API_ERROR_MODE);
		break;
	}
errors:
	RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_de_init(void) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	TIM_status_t tim_status = TIM_SUCCESS;
	POWER_status_t power_status = POWER_SUCCESS;
	RFE_status_t rfe_status = RFE_SUCCESS;
	// Release DIO2.
	GPIO_write(&GPIO_SX1232_DIO2, 0);
	// Release DIO0.
	EXTI_release_gpio(&GPIO_SX1232_DIO0);
	GPIO_configure(&GPIO_SX1232_DIO0, GPIO_MODE_OUTPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Release symbol profile timer.
	tim_status = TIM_STD_de_init(RF_API_MODULATION_TIMER_INSTANCE);
	TIM_stack_exit_error(ERROR_BASE_TIM_MODULATION, (RF_API_status_t) RF_API_ERROR_DRIVER_TIMER_MODULATION);
	// Disable front-end.
	rfe_status = RFE_set_path(RFE_PATH_NONE);
	RFE_stack_exit_error(ERROR_BASE_RFE, (RF_API_status_t) RF_API_ERROR_DRIVER_RFE);
	// Turn radio off.
	power_status = POWER_disable(POWER_DOMAIN_RADIO);
	POWER_stack_exit_error(ERROR_BASE_POWER, (RF_API_status_t) RF_API_ERROR_DRIVER_POWER);
	RETURN();
errors:
	POWER_disable(POWER_DOMAIN_RADIO);
	RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_send(RF_API_tx_data_t *tx_data) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	TIM_status_t tim_status = TIM_SUCCESS;
	sfx_u8 idx = 0;
#ifdef PARAMETERS_CHECK
	// Check parameter.
	if (tx_data == SFX_NULL) {
		EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
	}
#endif
	// Store TX data.
	rf_api_ctx.tx_bitstream_size_bytes = (tx_data -> bitstream_size_bytes);
	for (idx=0 ; idx<(rf_api_ctx.tx_bitstream_size_bytes) ; idx++) {
		rf_api_ctx.tx_bitstream[idx] = (tx_data -> bitstream)[idx];
	}
	// Init state.
	rf_api_ctx.tx_bit_idx = 0;
	rf_api_ctx.tx_byte_idx = 0;
	rf_api_ctx.tx_symbol_profile_idx = 0;
	rf_api_ctx.state = RF_API_STATE_TX_START;
	rf_api_ctx.flags.all = 0;
	// Trigger TX.
	status = _RF_API_internal_process();
	CHECK_STATUS(RF_API_SUCCESS);
	// Start timer.
	tim_status = TIM_STD_start(RF_API_MODULATION_TIMER_INSTANCE);
	TIM_stack_exit_error(ERROR_BASE_TIM_MODULATION, (RF_API_status_t) RF_API_ERROR_DRIVER_TIMER_MODULATION);
	// Wait for transmission to complete.
	while (rf_api_ctx.state != RF_API_STATE_READY) {
		// Wait for GPIO interrupt.
		while (rf_api_ctx.flags.field.timer_irq_flag == 0);
		// Clear flag.
		rf_api_ctx.flags.field.timer_irq_flag = 0;
		// Call process function.
		status = _RF_API_internal_process();
		CHECK_STATUS(RF_API_SUCCESS);
	}
	tim_status = TIM_STD_stop(RF_API_MODULATION_TIMER_INSTANCE);
	TIM_stack_exit_error(ERROR_BASE_TIM_MODULATION, (RF_API_status_t) RF_API_ERROR_DRIVER_TIMER_MODULATION);
errors:
	RETURN();
}

#ifdef BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t RF_API_receive(RF_API_rx_data_t *rx_data) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	sfx_bool dl_timeout = SFX_FALSE;
#ifdef PARAMETERS_CHECK
	// Check parameter.
	if (rx_data == SFX_NULL) {
		EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
	}
#endif
	// Enable GPIO interrupt.
	EXTI_clear_gpio_flag(&GPIO_SX1232_DIO0);
	EXTI_enable_gpio_interrupt(&GPIO_SX1232_DIO0);
	// Reset flag.
	(rx_data -> data_received) = SFX_FALSE;
	// Init state.
	rf_api_ctx.state = RF_API_STATE_RX_START;
	rf_api_ctx.flags.all = 0;
	// Trigger RX.
	status = _RF_API_internal_process();
	CHECK_STATUS(RF_API_SUCCESS);
	// Wait for reception to complete.
	while (rf_api_ctx.state != RF_API_STATE_READY) {
		// Wait for GPIO interrupt.
		while (rf_api_ctx.flags.field.gpio_irq_flag == 0) {
			// Enter sleep mode.
			PWR_enter_sleep_mode();
			IWDG_reload();
			// Check timeout.
			mcu_api_status = MCU_API_timer_status(MCU_API_TIMER_INSTANCE_T_RX, &dl_timeout);
			MCU_API_check_status((RF_API_status_t) RF_API_ERROR_DRIVER_MCU_API);
			// Exit if timeout.
			if (dl_timeout == SFX_TRUE) {
				// Stop radio.
				sx1232_status = SX1232_set_mode(SX1232_MODE_STANDBY);
				SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
				// Exit loop.
				goto errors;
			}
		}
		// Clear flag.
		rf_api_ctx.flags.field.gpio_irq_flag = 0;
		// Call process function.
		status = _RF_API_internal_process();
		CHECK_STATUS(RF_API_SUCCESS);
	}
	// Update status flag.
	(rx_data -> data_received) = SFX_TRUE;
errors:
	// Disable GPIO interrupt.
	EXTI_disable_gpio_interrupt(&GPIO_SX1232_DIO0);
	RETURN();
}
#endif

#ifdef BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t RF_API_get_dl_phy_content_and_rssi(sfx_u8 *dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16 *dl_rssi_dbm) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	sfx_u8 idx = 0;
#ifdef PARAMETERS_CHECK
	// Check parameters.
	if ((dl_phy_content == SFX_NULL) || (dl_rssi_dbm == SFX_NULL)) {
		EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
	}
	if (dl_phy_content_size > SIGFOX_DL_PHY_CONTENT_SIZE_BYTES) {
		EXIT_ERROR((RF_API_status_t) RF_API_ERROR_BUFFER_SIZE);
	}
#endif
	// Fill data.
	for (idx=0 ; idx<dl_phy_content_size ; idx++) {
		dl_phy_content[idx] = rf_api_ctx.dl_phy_content[idx];
	}
	(*dl_rssi_dbm) = (sfx_s16) rf_api_ctx.dl_rssi_dbm;
errors:
	RETURN();
}
#endif

#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_LBT)
/*******************************************************************/
RF_API_status_t RF_API_carrier_sense(RF_API_carrier_sense_parameters_t *carrier_sense_params) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	RETURN();
}
#endif

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
/*******************************************************************/
RF_API_status_t RF_API_get_latency(RF_API_latency_t latency_type, sfx_u32 *latency_ms) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
#ifdef PARAMETERS_CHECK
	// Check parameter.
	if (latency_ms == SFX_NULL) {
		EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
	}
	if (latency_type >= RF_API_LATENCY_LAST) {
		EXIT_ERROR((RF_API_status_t) RF_API_ERROR_LATENCY_TYPE);
	}
#endif
	// Set latency.
	(*latency_ms) = RF_API_LATENCY_MS[latency_type];
#ifdef PARAMETERS_CHECK
errors:
#endif
	RETURN();
}
#endif

#ifdef CERTIFICATION
/*******************************************************************/
RF_API_status_t RF_API_start_continuous_wave(void) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	// Start radio.
	sx1232_status = SX1232_start_tx();
	SX1232_stack_exit_error(ERROR_BASE_SX1232, (RF_API_status_t) RF_API_ERROR_DRIVER_SX1232);
	// Start OOK data signal.
	GPIO_write(&GPIO_SX1232_DIO2, 1);
errors:
	RETURN();
}
#endif

#ifdef VERBOSE
/*******************************************************************/
RF_API_status_t RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	RETURN();
}
#endif

#ifdef ERROR_CODES
/*******************************************************************/
void RF_API_error(void) {
	POWER_disable(POWER_DOMAIN_RADIO);
}
#endif
