/*
 * rf_api.c
 *
 *  Created on: 18 juin 2018
 *      Author: Ludo
 */

#include "rf_api.h"

#include "gpio.h"
#include "iwdg.h"
#include "lptim.h"
#include "mapping.h"
#include "mode.h"
#include "nvic.h"
#include "rtc.h"
#include "sigfox_api.h"
#include "sigfox_types.h"
#include "sky13317.h"
#include "spi.h"
#include "sx1232.h"
#include "tim.h"
#include "tim_reg.h"
#include "usart.h"

/*** RF API local macros ***/

// Uplink parameters.
#define RF_API_UPLINK_OUTPUT_POWER_ETSI		14
#define RF_API_UPLINK_OUTPUT_POWER_FCC		20
// Downlink parameters.
#define RF_API_DOWNLINK_FRAME_LENGTH_BYTES	15
#define RF_API_DOWNLINK_TIMEOUT_SECONDS		25
#define RF_API_WAIT_FRAME_CALLS_MAX			100

/*** RF API local structures ***/

// Sigfox uplink modulation parameters.
typedef struct {
	// Uplink message frequency.
	unsigned int rf_api_rf_frequency_hz;
	// Modulation parameters.
	unsigned short rf_api_symbol_duration_us;
	volatile unsigned char rf_api_tim2_arr_flag;
	unsigned short rf_api_ramp_duration_us;
	volatile unsigned char rf_api_phase_shift_required;
	unsigned int rf_api_frequency_shift_hz;
	volatile unsigned char rf_api_frequency_shift_direction;
	// Output power range.
	signed char rf_api_output_power_max;
	// Downlink.
	unsigned int rf_api_wait_frame_calls_count;
} RF_api_context_t;

/*** RF API local global variables ***/

static RF_api_context_t rf_api_ctx;

/*** RF API local functions ***/

/* TIM2 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
 void __attribute__((optimize("-O0"))) TIM2_IRQHandler(void) {
	// ARR = symbol rate.
	if (((TIM2 -> SR) & (0b1 << TIM2_TIMINGS_ARRAY_ARR_IDX)) != 0) {
		// Update ARR flag.
		rf_api_ctx.rf_api_tim2_arr_flag = 1;
		// Clear flag.
		TIM2 -> SR &= ~(0b1 << TIM2_TIMINGS_ARRAY_ARR_IDX);
	}
	// CCR1 = ramp down start.
	else if (((TIM2 -> SR) & (0b1 << TIM2_TIMINGS_ARRAY_CCR1_IDX)) != 0) {
		// Update ARR flag.
		rf_api_ctx.rf_api_tim2_arr_flag = 0;
		// Clear flag.
		TIM2 -> SR &= ~(0b1 << TIM2_TIMINGS_ARRAY_CCR1_IDX);
		if (rf_api_ctx.rf_api_phase_shift_required != 0) {
			// Turn signal off (ramp down is done by the transceiver OOK modulation shaping).
			GPIO_write(&GPIO_SX1232_DIO2, 0);
		}
	}
	// CCR2 = ramp down end + frequency shift start.
	else if (((TIM2 -> SR) & (0b1 << TIM2_TIMINGS_ARRAY_CCR2_IDX)) != 0) {
		if (rf_api_ctx.rf_api_phase_shift_required != 0) {
			// Change frequency.
			if (rf_api_ctx.rf_api_frequency_shift_direction == 0) {
				// Decrease frequency.
				SX1232_set_rf_frequency(rf_api_ctx.rf_api_rf_frequency_hz - rf_api_ctx.rf_api_frequency_shift_hz);
				rf_api_ctx.rf_api_frequency_shift_direction = 1;
			}
			else {
				// Increase frequency.
				SX1232_set_rf_frequency(rf_api_ctx.rf_api_rf_frequency_hz + rf_api_ctx.rf_api_frequency_shift_hz);
				rf_api_ctx.rf_api_frequency_shift_direction = 0;
			}
		}
		// Update ARR flag.
		rf_api_ctx.rf_api_tim2_arr_flag = 0;
		// Clear flag.
		TIM2 -> SR &= ~(0b1 << TIM2_TIMINGS_ARRAY_CCR2_IDX);
	}
	// CCR3 = frequency shift end + ramp-up start.
	else if (((TIM2 -> SR) & (0b1 << TIM2_TIMINGS_ARRAY_CCR3_IDX)) != 0) {
		if (rf_api_ctx.rf_api_phase_shift_required != 0){
			// Come back to uplink frequency.
			SX1232_set_rf_frequency(rf_api_ctx.rf_api_rf_frequency_hz);
			// Turn signal on (ramp up is done by the transceiver OOK modulation shaping).
			GPIO_write(&GPIO_SX1232_DIO2, 1);
		}
		// Update ARR flag.
		rf_api_ctx.rf_api_tim2_arr_flag = 0;
		// Clear flag.
		TIM2 -> SR &= ~(0b1 << TIM2_TIMINGS_ARRAY_CCR3_IDX);
	}
	// CCR4 = ramp-up end.
	else if (((TIM2 -> SR) & (0b1 << TIM2_TIMINGS_ARRAY_CCR4_IDX)) != 0) {
		// Update ARR flag.
		rf_api_ctx.rf_api_tim2_arr_flag = 0;
		// Clear flag.
		TIM2 -> SR &= ~(0b1 << TIM2_TIMINGS_ARRAY_CCR4_IDX);
	}
}

/* UPDATE PARAMETERS ACCORDING TO MODULATION TYPE.
 * @param modulation:	Modulation type asked by Sigfox library.
 * @return status:		Function execution status.
 */
static SIGFOX_API_status_t RF_API_SetTxModulationParameters(sfx_modulation_type_t modulation) {
	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	// Init common parameters.
	rf_api_ctx.rf_api_phase_shift_required = 0;
	rf_api_ctx.rf_api_frequency_shift_direction = 0;
	// Init timings.
	switch (modulation) {
	case SFX_DBPSK_100BPS:
		// 100 bps timings.
		rf_api_ctx.rf_api_symbol_duration_us = 10000;
		rf_api_ctx.rf_api_frequency_shift_hz = 400;
		rf_api_ctx.rf_api_output_power_max = RF_API_UPLINK_OUTPUT_POWER_ETSI;
		break;
	case SFX_DBPSK_600BPS:
		// 600 bps timings.
		rf_api_ctx.rf_api_symbol_duration_us = 1667;
		rf_api_ctx.rf_api_frequency_shift_hz = 2000;
		rf_api_ctx.rf_api_output_power_max = RF_API_UPLINK_OUTPUT_POWER_FCC;
		break;
	default:
		status = SIGFOX_API_ERROR_MODULATION_TYPE;
		goto errors;
	}
	rf_api_ctx.rf_api_ramp_duration_us = (rf_api_ctx.rf_api_symbol_duration_us / 4);
errors:
	return status;
}

/* SELECT RF PATH ACCORDING TO RF MODE AND SIGFOX RADIO CONFIGURAION.
 * @param:	None.
 * @return:	None.
 */
static void RF_API_SetRfPath(sfx_rf_mode_t rf_mode) {
	// Select TX / RX.
	switch (rf_mode) {
	case SFX_RF_MODE_TX:
		// Select TX path.
		SX1232_set_rf_output_pin(SX1232_RF_OUTPUT_PIN_PABOOST);
#ifdef HW1_0
		SKY13317_set_channel(SKY13317_CHANNEL_RF1);
#endif
#ifdef HW2_0
		SKY13317_set_channel(SKY13317_CHANNEL_RF2);
#endif
		break;
	case SFX_RF_MODE_RX:
		// Activate LNA path.
#ifdef HW1_0
		SKY13317_set_channel(SKY13317_CHANNEL_RF2);
#endif
#ifdef HW2_0
		SKY13317_set_channel(SKY13317_CHANNEL_RF3);
#endif
		break;
	default:
		// Disable every channel.
		SKY13317_set_channel(SKY13317_CHANNEL_NONE);
		break;
	}
}

/*** RF API functions ***/

/*!******************************************************************
 * \fn sfx_u8 RF_API_init(sfx_rf_mode_t rf_mode)
 * \brief Init and configure Radio link in RX/TX
 *
 * [RX Configuration]
 * To receive Sigfox Frame on your device, program the following:
 *  - Preamble  : 0xAAAAAAAAA
 *  - Sync Word : 0xB227
 *  - Packet of the Sigfox frame is 15 bytes length.
 *
 * \param[in] sfx_rf_mode_t rf_mode         Init Radio link in Tx or RX
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:             No error
 * \retval RF_ERR_API_INIT:          Init Radio link error
 *******************************************************************/
sfx_u8 RF_API_init(sfx_rf_mode_t rf_mode) {
	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	SPI_status_t spi1_status = SPI_SUCCESS;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	// Switch RF on.
	spi1_status = SPI1_power_on();
	SPI1_status_check(SIGFOX_API_ERROR_BASE_SPI);
	// Init transceiver.
	sx1232_status = SX1232_set_oscillator(SX1232_OSCILLATOR_TCXO);
	SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
	// Configure switch.
	RF_API_SetRfPath(rf_mode);
	// Configure transceiver.
	unsigned char downlink_sync_word[2] = {0xB2, 0x27};
	switch (rf_mode) {
	// Uplink.
	case SFX_RF_MODE_TX:
		// Prepare transceiver for uplink or continuous wave operation.
		sx1232_status = SX1232_enable_low_phase_noise_pll();
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		sx1232_status = SX1232_enable_fast_frequency_hopping();
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		sx1232_status = SX1232_set_modulation(SX1232_MODULATION_OOK, SX1232_MODULATION_SHAPING_OOK_BITRATE);
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		sx1232_status = SX1232_set_bitrate(500); // Set bit rate for modulation shaping.
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		sx1232_status = SX1232_set_data_mode(SX1232_DATA_MODE_CONTINUOUS);
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		break;
	// Downlink.
	case SFX_RF_MODE_RX:
		// Reset call counter.
		rf_api_ctx.rf_api_wait_frame_calls_count = 0;
		// Prepare transceiver for downlink operation (GFSK 800Hz 600bps).
		sx1232_status = SX1232_set_modulation(SX1232_MODULATION_FSK, SX1232_MODULATION_SHAPING_FSK_BT_1);
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		sx1232_status = SX1232_set_fsk_deviation(800);
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		sx1232_status = SX1232_set_bitrate(600);
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		sx1232_status = SX1232_set_rx_bandwidth(SX1232_RXBW_MANTISSA_24, SX1232_RXBW_EXPONENT_7);
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		sx1232_status = SX1232_enable_lna_boost(1);
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		sx1232_status = SX1232_configure_rssi(SX1232_RSSI_SAMPLING_32);
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		sx1232_status = SX1232_set_preamble_detector(1, 0);
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		sx1232_status = SX1232_set_sync_word(downlink_sync_word, 2);
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		sx1232_status = SX1232_set_data_length(RF_API_DOWNLINK_FRAME_LENGTH_BYTES);
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		sx1232_status = SX1232_set_dio_mapping(SX1232_DIO0, SX1232_DIO_MAPPING0); // Map payload ready interrupt on DIO0.
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		sx1232_status = SX1232_set_data_mode(SX1232_DATA_MODE_PACKET);
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		break;
	default:
		status = SIGFOX_API_ERROR_RF_MODE;
		goto errors;
	}
errors:
	return status;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_stop(void)
 * \brief Close Radio link
 *
 * \param[in] none
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:              No error
 * \retval RF_ERR_API_STOP:           Close Radio link error
 *******************************************************************/
sfx_u8 RF_API_stop(void) {
	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	SPI_status_t spi1_status = SPI_SUCCESS;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	// Disable all switch channels.
	SKY13317_set_channel(SKY13317_CHANNEL_NONE);
	// Power transceiver down.
	sx1232_status = SX1232_set_mode(SX1232_MODE_STANDBY);
	SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
	spi1_status = SPI1_power_off();
	SPI1_status_check(SIGFOX_API_ERROR_BASE_SPI);
errors:
	return status;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_send(sfx_u8 *stream, sfx_modulation_type_t type, sfx_u8 size)
 * \brief BPSK Modulation of data stream
 * (from synchro bit field to CRC)
 *
 * NOTE : during this function, the voltage_tx needs to be retrieved and stored in
 *        a variable to be returned into the MCU_API_get_voltage_and_temperature or
 *        MCU_API_get_voltage functions.
 *
 * \param[in] sfx_u8 *stream                Complete stream to modulate
 * \param[in]sfx_modulation_type_t          Type of the modulation ( enum with baudrate and modulation information)
 * \param[in] sfx_u8 size                   Length of stream
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                    No error
 * \retval RF_ERR_API_SEND:                 Send data stream error
 *******************************************************************/
sfx_u8 RF_API_send(sfx_u8 *stream, sfx_modulation_type_t type, sfx_u8 size) {
	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	unsigned char stream_byte_idx = 0;
	unsigned char stream_bit_idx = 0;
	unsigned int effective_uplink_frequency_hz = 0;
	unsigned int effective_high_shifted_frequency_hz = 0;
	unsigned int effective_low_shifted_frequency_hz = 0;
	unsigned short high_shifted_frequency_duration_us = 0;
	unsigned short low_shifted_frequency_duration_us = 0;
	unsigned short frequency_shift_duration_us = 0;
	unsigned short high_shifted_idle_duration_us = 0;
	unsigned short low_shifted_idle_duration_us = 0;
	unsigned short idle_duration_us = 0;
	unsigned short dbpsk_timings[TIM2_TIMINGS_ARRAY_LENGTH] = {0};
	// Disable all interrupts.
#ifdef ATM
#ifdef HW1_0
	NVIC_disable_interrupt(NVIC_IT_USART2);
#endif
#ifdef HW2_0
	NVIC_disable_interrupt(NVIC_IT_USART1);
#endif
#endif
	NVIC_disable_interrupt(NVIC_IT_LPTIM1);
	NVIC_disable_interrupt(NVIC_IT_RTC);
	NVIC_disable_interrupt(NVIC_IT_DMA1_CH_4_7);
	NVIC_disable_interrupt(NVIC_IT_EXTI_4_15);
	NVIC_disable_interrupt(NVIC_IT_LPUART1);
	// Set modulation parameters.
	status = RF_API_SetTxModulationParameters(type);
	if (status != SIGFOX_API_SUCCESS) goto errors;
	// Compute frequency shift duration required to invert signal phase.
	// Compensate transceiver synthetizer step by programming and reading effective frequencies.
	sx1232_status = SX1232_set_rf_frequency(rf_api_ctx.rf_api_rf_frequency_hz);
	SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
	sx1232_status = SX1232_get_rf_frequency(&effective_uplink_frequency_hz);
	SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
	sx1232_status = SX1232_set_rf_frequency(rf_api_ctx.rf_api_rf_frequency_hz + rf_api_ctx.rf_api_frequency_shift_hz);
	SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
	sx1232_status = SX1232_get_rf_frequency(&effective_high_shifted_frequency_hz);
	SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
	sx1232_status = SX1232_set_rf_frequency(rf_api_ctx.rf_api_rf_frequency_hz - rf_api_ctx.rf_api_frequency_shift_hz);
	SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
	sx1232_status = SX1232_get_rf_frequency(&effective_low_shifted_frequency_hz);
	SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
	// Compute average durations = 1 / (2 * delta_f).
	high_shifted_frequency_duration_us = (1000000) / (2 * (effective_high_shifted_frequency_hz - effective_uplink_frequency_hz));
	low_shifted_frequency_duration_us = (1000000) / (2 * (effective_uplink_frequency_hz - effective_low_shifted_frequency_hz));
	frequency_shift_duration_us = (high_shifted_frequency_duration_us + low_shifted_frequency_duration_us) / (2);
	// Compute average idle duration (before and after signal phase inversion).
	high_shifted_idle_duration_us = (rf_api_ctx.rf_api_symbol_duration_us - (2 * rf_api_ctx.rf_api_ramp_duration_us) - (high_shifted_frequency_duration_us)) / (2);
	low_shifted_idle_duration_us = (rf_api_ctx.rf_api_symbol_duration_us - (2 * rf_api_ctx.rf_api_ramp_duration_us) - (low_shifted_frequency_duration_us)) / (2);
	idle_duration_us = (high_shifted_idle_duration_us + low_shifted_idle_duration_us) / (2);
	// Configure timer.
	dbpsk_timings[TIM2_TIMINGS_ARRAY_ARR_IDX] = rf_api_ctx.rf_api_symbol_duration_us;
	dbpsk_timings[TIM2_TIMINGS_ARRAY_CCR1_IDX] = idle_duration_us;
	dbpsk_timings[TIM2_TIMINGS_ARRAY_CCR2_IDX] = dbpsk_timings[1] + rf_api_ctx.rf_api_ramp_duration_us;
	dbpsk_timings[TIM2_TIMINGS_ARRAY_CCR3_IDX] = dbpsk_timings[2] + frequency_shift_duration_us;
	dbpsk_timings[TIM2_TIMINGS_ARRAY_CCR4_IDX] = dbpsk_timings[3] + rf_api_ctx.rf_api_ramp_duration_us;
	TIM2_init(dbpsk_timings);
	TIM2_enable();
	NVIC_enable_interrupt(NVIC_IT_TIM2);
	rf_api_ctx.rf_api_tim2_arr_flag = 0;
	// Start CW.
	sx1232_status = SX1232_set_rf_frequency(rf_api_ctx.rf_api_rf_frequency_hz);
	SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
	sx1232_status = SX1232_set_rf_output_power(rf_api_ctx.rf_api_output_power_max);
	SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
	sx1232_status = SX1232_start_cw();
	SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
	// First ramp-up.
	rf_api_ctx.rf_api_tim2_arr_flag = 0;
	rf_api_ctx.rf_api_phase_shift_required = 0;
	TIM2_start();
	// Data transmission.
	while (rf_api_ctx.rf_api_tim2_arr_flag == 0);
	// Byte loop.
	for (stream_byte_idx=0 ; stream_byte_idx<size ; stream_byte_idx++) {
		// Bit loop.
		for (stream_bit_idx=0 ; stream_bit_idx<8 ; stream_bit_idx++) {
			// Clear ARR flag.
			rf_api_ctx.rf_api_tim2_arr_flag = 0;
			// Phase shift required is bit is '0'.
			if ((stream[stream_byte_idx] & (0b1 << (7-stream_bit_idx))) == 0) {
				rf_api_ctx.rf_api_phase_shift_required = 1;
			}
			else {
				rf_api_ctx.rf_api_phase_shift_required = 0;
			}
			// Wait the end of symbol period.
			while (rf_api_ctx.rf_api_tim2_arr_flag == 0);
		}
	}
	// Last ramp down.
	rf_api_ctx.rf_api_tim2_arr_flag = 0;
	rf_api_ctx.rf_api_phase_shift_required = 0;
	GPIO_write(&GPIO_SX1232_DIO2, 0);
	while (rf_api_ctx.rf_api_tim2_arr_flag == 0);
	// Stop CW.
	sx1232_status = SX1232_stop_cw();
	SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
	TIM2_stop();
	TIM2_disable();
	// Re-enable all interrupts.
	NVIC_enable_interrupt(NVIC_IT_RTC);
#ifdef ATM
#ifdef HW1_0
	NVIC_enable_interrupt(NVIC_IT_USART2);
#endif
#ifdef HW2_0
	NVIC_enable_interrupt(NVIC_IT_USART1);
#endif
#endif
errors:
	return status;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_start_continuous_transmission (sfx_modulation_type_t type)
 * \brief Generate a signal with modulation type. All the configuration ( Init of the RF and Frequency have already been executed
 *        when this function is called.
 *
 * \param[in] sfx_modulation_type_t         Type of the modulation ( enum with baudrate and modulation information is contained in sigfox_api.h)
 *
 * \retval SFX_ERR_NONE:                                 No error
 * \retval RF_ERR_API_START_CONTINUOUS_TRANSMISSION:     Continuous Transmission Start error
 *******************************************************************/
sfx_u8 RF_API_start_continuous_transmission (sfx_modulation_type_t type) {
	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	// Start CW.
	sx1232_status = SX1232_set_rf_output_power(rf_api_ctx.rf_api_output_power_max);
	SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
	sx1232_status = SX1232_start_cw();
	SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
errors:
	return status;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_stop_continuous_transmission (void)
 * \brief Stop the current continuous transmisssion
 *
 * \retval SFX_ERR_NONE:                                 No error
 * \retval RF_ERR_API_STOP_CONTINUOUS_TRANSMISSION:      Continuous Transmission Stop error
 *******************************************************************/
sfx_u8 RF_API_stop_continuous_transmission (void) {
	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	// Stop CW.
	sx1232_status = SX1232_stop_cw();
	SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
errors:
	return status;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_change_frequency(sfx_u32 frequency)
 * \brief Change synthesizer carrier frequency
 *
 * \param[in] sfx_u32 frequency             Frequency in Hz to program in the radio chipset
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                    No error
 * \retval RF_ERR_API_CHANGE_FREQ:          Change frequency error
 *******************************************************************/
sfx_u8 RF_API_change_frequency(sfx_u32 frequency) {
	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	// Program and save frequency.
	sx1232_status = SX1232_set_rf_frequency(frequency);
	SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
	// Update local variable.
	rf_api_ctx.rf_api_rf_frequency_hz = frequency;
errors:
	return status;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_wait_frame(sfx_u8 *frame, sfx_s16 *rssi, sfx_rx_state_enum_t * state)
 * \brief Get all GFSK frames received in Rx buffer, structure of
 * frame is : Synchro bit + Synchro frame + 15 Bytes.<BR> This function must
 * be blocking state since data is received or timer of 25 s has elapsed.
 *
 * - If received buffer, function returns SFX_ERR_NONE then the
 *   library will try to decode frame. If the frame is not correct, the
 *   library will recall RF_API_wait_frame.
 *
 * - If 25 seconds timer has elapsed, function returns into the state the timeout enum code.
 *   and then library will stop receive frame phase.
 *
 * \param[in] none
 * \param[out] sfx_s8 *frame                  Receive buffer
 * \param[out] sfx_s16 *rssi                  Chipset RSSI
 * Warning: This is the 'raw' RSSI value. Do not add 100 as made
 * in Library versions 1.x.x
 * Resolution: 1 LSB = 1 dBm
 *
 * \param[out] sfx_rx_state_enum_t state      Indicate the final state of the reception. Value can be DL_TIMEOUT or DL_PASSED
 *                                            if a frame has been received, as defined in sigfox_api.h file.
 *
 * \retval SFX_ERR_NONE:                      No error
 *******************************************************************/
sfx_u8 RF_API_wait_frame(sfx_u8 *frame, sfx_s16 *rssi, sfx_rx_state_enum_t *state) {
	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	unsigned char rssi_retrieved = 0;
	unsigned int remaining_delay = RF_API_DOWNLINK_TIMEOUT_SECONDS;
	unsigned int sub_delay = 0;
	unsigned int irq_flags = 0;
	// Init state.
	(*state) = DL_TIMEOUT;
	// Manage call count.
	rf_api_ctx.rf_api_wait_frame_calls_count++;
	if (rf_api_ctx.rf_api_wait_frame_calls_count < RF_API_WAIT_FRAME_CALLS_MAX) {
		// Go to FSRX state.
		sx1232_status = SX1232_set_mode(SX1232_MODE_FSRX);
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		// Wait TS_FS=60us typical.
		lptim1_status = LPTIM1_delay_milliseconds(5, 1);
		LPTIM1_status_check(SIGFOX_API_ERROR_BASE_LPTIM);
		// Go to RX state.
		sx1232_status = SX1232_set_mode(SX1232_MODE_RX);
		SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		// Wait TS_TR=120us typical.
		lptim1_status = LPTIM1_delay_milliseconds(5, 1);
		LPTIM1_status_check(SIGFOX_API_ERROR_BASE_LPTIM);
		// Wait for external interrupt (payload ready on DIO0).
		while ((remaining_delay > 0) && (GPIO_read(&GPIO_SX1232_DIO0) == 0)) {
			// Compute sub-delay.
			sub_delay = (remaining_delay > IWDG_REFRESH_PERIOD_SECONDS) ? (IWDG_REFRESH_PERIOD_SECONDS) : (remaining_delay);
			remaining_delay -= sub_delay;
			// Start wake-up timer.
			RTC_clear_wakeup_timer_flag();
			rtc_status = RTC_start_wakeup_timer(sub_delay);
			RTC_status_check(SIGFOX_API_ERROR_BASE_RTC);
			while (RTC_get_wakeup_timer_flag() == 0) {
				// Read SX1232 IRQ flags.
				sx1232_status = SX1232_get_irq_flags(&irq_flags);
				SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
				// Get RSSI when preamble is found.
				if (((irq_flags & 0x0200) != 0) && (rssi_retrieved == 0)) {
					sx1232_status = SX1232_get_rssi(rssi);
					SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
					rssi_retrieved = 1;
				}
			}
			// Sub-delay reached: clear watchdog and flags.
			IWDG_reload();
		}
		// Stop timer.
		rtc_status = RTC_stop_wakeup_timer();
		RTC_status_check(SIGFOX_API_ERROR_BASE_RTC);
		// Check GPIO.
		if (GPIO_read(&GPIO_SX1232_DIO0) != 0) {
			// Downlink frame received.
			(*state) = DL_PASSED;
			sx1232_status = SX1232_read_fifo(frame, RF_API_DOWNLINK_FRAME_LENGTH_BYTES);
			SX1232_status_check(SIGFOX_API_ERROR_BASE_SX1232);
		}
	}
errors:
	return status;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_wait_for_clear_channel (sfx_u8 cs_min, sfx_s8 cs_threshold, sfx_rx_state_enum_t * state);
 * \brief This function is used in ARIB standard for the Listen Before Talk
 *        feature. It listens on a specific frequency band initialized through the RF_API_init(), during a sliding window set
 *        in the MCU_API_timer_start_carrier_sense().
 *        If the channel is clear during the minimum carrier sense
 *        value (cs_min), under the limit of the cs_threshold,
 *        the functions returns with SFX_ERR_NONE (transmission
 *        allowed). Otherwise it continues to listen to the channel till the expiration of the
 *        carrier sense maximum window and then updates the state ( with timeout enum ).
 *
 * \param[in] none
 * \param[out] sfx_u8 cs_min                  Minimum Carrier Sense time in ms.
 * \param[out] sfx_s8 cs_threshold            Power threshold limit to declare the channel clear.
 *                                            i.e : cs_threshold value -80dBm in Japan / -65dBm in Korea
 * \param[out] sfx_rx_state_enum_t state      Indicate the final state of the carrier sense. Value can be DL_TIMEOUT or PASSED
 *                                            as per defined in sigfox_api.h file.
 *
 * \retval SFX_ERR_NONE:                      No error
 *******************************************************************/
sfx_u8 RF_API_wait_for_clear_channel(sfx_u8 cs_min, sfx_s8 cs_threshold, sfx_rx_state_enum_t * state) {
	return SIGFOX_API_SUCCESS;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_get_version(sfx_u8 **version, sfx_u8 *size)
 * \brief Returns current RF API version
 *
 * \param[out] sfx_u8 **version                 Pointer to Byte array (ASCII format) containing library version
 * \param[out] sfx_u8 *size                     Size of the byte array pointed by *version
 *
 * \retval SFX_ERR_NONE:                No error
 * \retval RF_ERR_API_GET_VERSION:      Get Version error
 *******************************************************************/
sfx_u8 RF_API_get_version(sfx_u8 **version, sfx_u8 *size) {
	return SIGFOX_API_SUCCESS;
}
