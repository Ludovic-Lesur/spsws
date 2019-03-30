/*
 * rf_api.c
 *
 *  Created on: 18 juin 2018
 *      Author: Ludo
 */

#include "rf_api.h"

#include "gpio.h"
#include "lptim.h"
#include "mapping.h"
#include "mode.h"
#include "nvic.h"
#include "sigfox_api.h"
#include "sigfox_types.h"
#include "sky13317.h"
#include "spi.h"
#include "sx1232.h"
#include "tim.h"
#include "tim_reg.h"
#include "usart.h"

/*** RF API local macros ***/

// If defined, print the Sigfox bit stream on UART.
//#define RF_API_LOG_FRAME

// Uplink parameters.
#define RF_API_UPLINK_OUTPUT_POWER_ETSI		14
#define RF_API_UPLINK_OUTPUT_POWER_FCC		22

// Downlink parameters.
#define RF_API_DOWNLINK_FRAME_LENGTH_BYTES	15
#define RF_API_DOWNLINK_TIMEOUT_SECONDS		25

/*** RF API local structures ***/

// Sigfox uplink modulation parameters.
typedef struct {
	// Uplink message frequency.
	unsigned int rf_api_rf_frequency_hz;
	// Modulation parameters.
	unsigned short rf_api_symbol_duration_us;
	volatile unsigned char rf_api_tim2_event_mask; // Read as [x x x CCR4 CCR3 CCR2 CCR1 ARR].
	unsigned short rf_api_ramp_duration_us;
	volatile unsigned char rf_api_phase_shift_required;
	unsigned int rf_api_frequency_shift_hz;
	volatile unsigned char rf_api_frequency_shift_direction;
	// Output power range.
	signed char rf_api_output_power_min;
	signed char rf_api_output_power_max;
} RF_API_Context;

/*** RF API local global variables ***/

RF_API_Context rf_api_ctx;

/*** RF API local functions ***/

/* TIM2 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
 void TIM2_IRQHandler(void) {

	/* ARR = symbol rate */
	if (((TIM2 -> SR) & (0b1 << TIM2_TIMINGS_ARRAY_ARR_IDX)) != 0) {
		// Update event status (set current and clear previous).
		rf_api_ctx.rf_api_tim2_event_mask |= (0b1 << TIM2_TIMINGS_ARRAY_ARR_IDX);
		rf_api_ctx.rf_api_tim2_event_mask &= ~(0b1 << TIM2_TIMINGS_ARRAY_CCR4_IDX);
		// Clear flag.
		TIM2 -> SR &= ~(0b1 << TIM2_TIMINGS_ARRAY_ARR_IDX);
	}

	/* CCR1 = ramp down start */
	else if (((TIM2 -> SR) & (0b1 << TIM2_TIMINGS_ARRAY_CCR1_IDX)) != 0) {
		// Update event status (set current and clear previous).
		rf_api_ctx.rf_api_tim2_event_mask |= (0b1 << TIM2_TIMINGS_ARRAY_CCR1_IDX);
		rf_api_ctx.rf_api_tim2_event_mask &= ~(0b1 << TIM2_TIMINGS_ARRAY_ARR_IDX);
		// Clear flag.
		TIM2 -> SR &= ~(0b1 << TIM2_TIMINGS_ARRAY_CCR1_IDX);
	}

	/* CCR2 = ramp down end + frequency shift start */
	else if (((TIM2 -> SR) & (0b1 << TIM2_TIMINGS_ARRAY_CCR2_IDX)) != 0) {
		if (rf_api_ctx.rf_api_phase_shift_required != 0) {
			// Switch radio off.
#ifdef HW1_0
			GPIO_Write(GPIO_SX1232_DIOX, 0);
#endif
#ifdef HW2_0
			GPIO_Write(GPIO_SX1232_DIO2, 0);
#endif
			// Change frequency.
			if (rf_api_ctx.rf_api_frequency_shift_direction == 0) {
				// Decrease frequency.
				SX1232_SetRfFrequency(rf_api_ctx.rf_api_rf_frequency_hz - rf_api_ctx.rf_api_frequency_shift_hz);
				rf_api_ctx.rf_api_frequency_shift_direction = 1;
			}
			else {
				// Increase frequency.
				SX1232_SetRfFrequency(rf_api_ctx.rf_api_rf_frequency_hz + rf_api_ctx.rf_api_frequency_shift_hz);
				rf_api_ctx.rf_api_frequency_shift_direction = 0;
			}
		}
		// Update event status (set current and clear previous).
		rf_api_ctx.rf_api_tim2_event_mask |= (0b1 << TIM2_TIMINGS_ARRAY_CCR2_IDX);
		rf_api_ctx.rf_api_tim2_event_mask &= ~(0b1 << TIM2_TIMINGS_ARRAY_CCR1_IDX);
		// Clear flag.
		TIM2 -> SR &= ~(0b1 << TIM2_TIMINGS_ARRAY_CCR2_IDX);
	}

	/* CCR3 = frequency shift end + ramp-up start */
	else if (((TIM2 -> SR) & (0b1 << TIM2_TIMINGS_ARRAY_CCR3_IDX)) != 0) {
		if (rf_api_ctx.rf_api_phase_shift_required != 0){
			// Come back to uplink frequency.
			SX1232_SetRfFrequency(rf_api_ctx.rf_api_rf_frequency_hz);
			// Switch radio on.
#ifdef HW1_0
			GPIO_Write(GPIO_SX1232_DIOX, 1);
#endif
#ifdef HW2_0
			GPIO_Write(GPIO_SX1232_DIO2, 1);
#endif
		}
		// Update event status (set current and clear previous).
		rf_api_ctx.rf_api_tim2_event_mask |= (0b1 << TIM2_TIMINGS_ARRAY_CCR3_IDX);
		rf_api_ctx.rf_api_tim2_event_mask &= ~(0b1 << TIM2_TIMINGS_ARRAY_CCR2_IDX);
		// Clear flag.
		TIM2 -> SR &= ~(0b1 << TIM2_TIMINGS_ARRAY_CCR3_IDX);
	}

	/* CCR4 = ramp-up end */
	else if (((TIM2 -> SR) & (0b1 << TIM2_TIMINGS_ARRAY_CCR4_IDX)) != 0) {
		// Update event status.
		rf_api_ctx.rf_api_tim2_event_mask |= (0b1 << TIM2_TIMINGS_ARRAY_CCR4_IDX);
		rf_api_ctx.rf_api_tim2_event_mask &= ~(0b1 << TIM2_TIMINGS_ARRAY_CCR3_IDX);
		// Clear flag.
		TIM2 -> SR &= ~(0b1 << TIM2_TIMINGS_ARRAY_CCR4_IDX);
	}
}

/* UPDATE PARAMETERS ACCORDING TO MODULATION TYPE.
 * @param modulation:	Modulation type asked by Sigfox library.
 * @return:				None.
 */
void RF_API_SetTxModulationParameters(sfx_modulation_type_t modulation) {

	/* Init common parameters */
	rf_api_ctx.rf_api_phase_shift_required = 0;
	rf_api_ctx.rf_api_frequency_shift_direction = 0;

	/* Init timings */
	switch (modulation) {
	case SFX_DBPSK_100BPS:
		// 100 bps timings.
		rf_api_ctx.rf_api_symbol_duration_us = 10000;
		rf_api_ctx.rf_api_ramp_duration_us = 2500;
		rf_api_ctx.rf_api_frequency_shift_hz = 400;
		break;
	case SFX_DBPSK_600BPS:
		// 600 bps timings.
		rf_api_ctx.rf_api_symbol_duration_us = 1667;
		rf_api_ctx.rf_api_ramp_duration_us = 0; // TBD.
		rf_api_ctx.rf_api_frequency_shift_hz = 0; // TBD.
		break;
	default:
		break;
	}
}

/* SELECT RF PATH ACCORDING TO RF MODE AND SIGFOX RADIO CONFIGURAION.
 * @param:	None.
 * @return:	None.
 */
void RF_API_SetRfPath(sfx_rf_mode_t rf_mode) {

	/* Select TX / RX */
	switch (rf_mode) {

	case SFX_RF_MODE_TX:
		// Select TX path.
		SX1232_SelectRfOutputPin(SX1232_RF_OUTPUT_PIN_PABOOST);
#ifdef HW1_0
		SKY13317_SetChannel(SKY13317_CHANNEL_RF1);
#endif
#ifdef HW2_0
		SKY13317_SetChannel(SKY13317_CHANNEL_RF2);
#endif
		rf_api_ctx.rf_api_output_power_min = SX1232_OUTPUT_POWER_PABOOST_MIN;
		rf_api_ctx.rf_api_output_power_max = RF_API_UPLINK_OUTPUT_POWER_ETSI;
		break;

	case SFX_RF_MODE_RX:
		// Activate LNA path.
#ifdef HW1_0
		SKY13317_SetChannel(SKY13317_CHANNEL_RF2);
#endif
#ifdef HW2_0
		SKY13317_SetChannel(SKY13317_CHANNEL_RF3);
#endif
		break;

	default:
		// Disable every channel.
		SKY13317_SetChannel(SKY13317_CHANNEL_NONE);
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

	/* Switch RF on and init transceiver */
	SPI1_Enable();
	SPI1_PowerOn();
	unsigned char downlink_sync_word[2] = {0xB2, 0x27};

	/* Enable TCXO */
	SX1232_SetOscillator(SX1232_OSCILLATOR_TCXO);

	/* Configure switch */
	RF_API_SetRfPath(rf_mode);

	/* Configure transceiver */
	switch (rf_mode) {

	// Uplink.
	case SFX_RF_MODE_TX:
		// Prepare transceiver for uplink or continuous wave operation.
		SX1232_EnableLowPnPll();
		SX1232_EnableFastFrequencyHopping();
		SX1232_SetModulation(SX1232_MODULATION_OOK, SX1232_MODULATION_SHAPING_OOK_BITRATE);
		SX1232_SetBitRate(4800); // Set bit rate for modulation shaping.
		SX1232_SetDataMode(SX1232_DATA_MODE_CONTINUOUS);
		break;

	// Downlink.
	case SFX_RF_MODE_RX:
		// Prepare transceiver for downlink operation (GFSK 800Hz 600bps).
		SX1232_SetModulation(SX1232_MODULATION_FSK, SX1232_MODULATION_SHAPING_FSK_BT_1);
		SX1232_SetFskDeviation(800);
		SX1232_SetBitRate(600);
		SX1232_SetRxBandwidth(SX1232_RXBW_MANTISSA_24, SX1232_RXBW_EXPONENT_MAX);
		SX1232_EnableLnaBoost(1);
		SX1232_ConfigureRssi(SX1232_RSSI_OFFSET_DB, SX1232_RSSI_SAMPLING_32);
		SX1232_SetPreambleDetector(1, 0);
		SX1232_SetSyncWord(downlink_sync_word, 2);
		SX1232_SetDataLength(RF_API_DOWNLINK_FRAME_LENGTH_BYTES);
		SX1232_SetDioMapping(0, 0); // Map payload ready interrupt on DIO0.
		SX1232_SetDataMode(SX1232_DATA_MODE_PACKET);
		break;

	default:
		break;
	}
	return SFX_ERR_NONE;
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

	/* Disable all switch channels */
	SKY13317_SetChannel(SKY13317_CHANNEL_NONE);

	/* Power transceiver down */
	SX1232_SetMode(SX1232_MODE_STANDBY);
	SPI1_PowerOff();
	SPI1_Disable();

	return SFX_ERR_NONE;
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

	/* Disable all interrupts */
	NVIC_DisableInterrupt(IT_RTC);
	NVIC_DisableInterrupt(IT_EXTI_0_1);
	NVIC_DisableInterrupt(IT_EXTI_2_3);
	NVIC_DisableInterrupt(IT_EXTI_4_15);
	NVIC_DisableInterrupt(IT_TIM21);
	NVIC_DisableInterrupt(IT_USART2);
	NVIC_DisableInterrupt(IT_LPUART1);

	/* Set modulation parameters */
	RF_API_SetTxModulationParameters(type);

	/* Init common variables */
	volatile unsigned int start_time = 0;
	unsigned char output_power_idx = 0;
	unsigned char stream_byte_idx = 0;
	unsigned char stream_bit_idx = 0;
#ifdef RF_API_LOG_FRAME
	// Frame buffer (maximum length is 26 bytes in Sigfox V1 protocol).
	unsigned char sfx_frame[26] = {0x00};
#endif

	/* Compute ramp steps */
	unsigned char output_power_dynamic = (rf_api_ctx.rf_api_output_power_max - rf_api_ctx.rf_api_output_power_min) + 1;
	unsigned int first_last_ramp_step_duration_us = (rf_api_ctx.rf_api_symbol_duration_us - 1000) / output_power_dynamic;
	unsigned int ramp_step_duration_us = (rf_api_ctx.rf_api_ramp_duration_us) / output_power_dynamic;

	/* Compute frequency shift duration required to invert signal phase */
	// Compensate transceiver synthetizer step by programming and reading effective frequencies.
	SX1232_SetRfFrequency(rf_api_ctx.rf_api_rf_frequency_hz);
	unsigned int effective_uplink_frequency_hz = SX1232_GetRfFrequency();
	SX1232_SetRfFrequency(rf_api_ctx.rf_api_rf_frequency_hz + rf_api_ctx.rf_api_frequency_shift_hz);
	unsigned int effective_high_shifted_frequency_hz = SX1232_GetRfFrequency();
	SX1232_SetRfFrequency(rf_api_ctx.rf_api_rf_frequency_hz - rf_api_ctx.rf_api_frequency_shift_hz);
	unsigned int effective_low_shifted_frequency_hz = SX1232_GetRfFrequency();
	// Compute average durations = 1 / (2 * delta_f).
	unsigned short high_shifted_frequency_duration_us = (1000000) / (2 * (effective_high_shifted_frequency_hz - effective_uplink_frequency_hz));
	unsigned short low_shifted_frequency_duration_us = (1000000) / (2 * (effective_uplink_frequency_hz - effective_low_shifted_frequency_hz));
	unsigned short frequency_shift_duration_us = (high_shifted_frequency_duration_us + low_shifted_frequency_duration_us) / (2);
	// Compute average idle duration (before and after signal phase inversion).
	unsigned short high_shifted_idle_duration_us = (rf_api_ctx.rf_api_symbol_duration_us - (2 * rf_api_ctx.rf_api_ramp_duration_us) - (high_shifted_frequency_duration_us)) / (2);
	unsigned short low_shifted_idle_duration_us = (rf_api_ctx.rf_api_symbol_duration_us - (2 * rf_api_ctx.rf_api_ramp_duration_us) - (low_shifted_frequency_duration_us)) / (2);
	unsigned short idle_duration_us = (high_shifted_idle_duration_us + low_shifted_idle_duration_us) / (2);

	/* Configure timer */
	unsigned short dbpsk_timings[TIM2_TIMINGS_ARRAY_LENGTH] = {0};
	dbpsk_timings[TIM2_TIMINGS_ARRAY_ARR_IDX] = rf_api_ctx.rf_api_symbol_duration_us;
	dbpsk_timings[TIM2_TIMINGS_ARRAY_CCR1_IDX] = idle_duration_us;
	dbpsk_timings[TIM2_TIMINGS_ARRAY_CCR2_IDX] = dbpsk_timings[1] + rf_api_ctx.rf_api_ramp_duration_us;
	dbpsk_timings[TIM2_TIMINGS_ARRAY_CCR3_IDX] = dbpsk_timings[2] + frequency_shift_duration_us;
	dbpsk_timings[TIM2_TIMINGS_ARRAY_CCR4_IDX] = dbpsk_timings[3] + rf_api_ctx.rf_api_ramp_duration_us;
	TIM2_Init(TIM2_MODE_SIGFOX, dbpsk_timings);
	TIM2_Enable();
	rf_api_ctx.rf_api_tim2_event_mask = 0;

	/* Start CW */
	SX1232_SetRfFrequency(rf_api_ctx.rf_api_rf_frequency_hz);
	SX1232_SetRfOutputPower(rf_api_ctx.rf_api_output_power_min);
	SX1232_StartCw();
	TIM2_Start();

	/* First ramp-up */
	start_time = TIM2_GetCounter();
	for (output_power_idx=0 ; output_power_idx<output_power_dynamic ; output_power_idx++) {
		SX1232_SetRfOutputPower(rf_api_ctx.rf_api_output_power_min + output_power_idx); // Update output power.
		while (TIM2_GetCounter() < (start_time + (first_last_ramp_step_duration_us * (output_power_idx + 1)))); // Wait until step duration is reached.
	}
	// Wait the end of symbol period.
	while ((rf_api_ctx.rf_api_tim2_event_mask & (0b1 << TIM2_TIMINGS_ARRAY_ARR_IDX)) == 0);

	/* Data transmission */
	// Byte loop.
	for (stream_byte_idx=0 ; stream_byte_idx<size ; stream_byte_idx++) {
#ifdef RF_API_LOG_FRAME
		// Store byte.
		sfx_frame[stream_byte_idx] = stream[stream_byte_idx];
#endif
		// Bit loop.
		for (stream_bit_idx=0 ; stream_bit_idx<8 ; stream_bit_idx++) {
			// Clear ARR flag.
			rf_api_ctx.rf_api_tim2_event_mask &= ~(0b1 << TIM2_TIMINGS_ARRAY_ARR_IDX);
			// Phase shift required is bit is '0'.
			if ((stream[stream_byte_idx] & (0b1 << (7-stream_bit_idx))) == 0) {
				// Update flag.
				rf_api_ctx.rf_api_phase_shift_required = 1;
				// First idle period.
				while ((rf_api_ctx.rf_api_tim2_event_mask & (0b1 << TIM2_TIMINGS_ARRAY_CCR1_IDX)) == 0);
				// Ramp down.
				start_time = TIM2_GetCounter();
				for (output_power_idx=0 ; output_power_idx<output_power_dynamic ; output_power_idx++) {
					SX1232_SetRfOutputPower(rf_api_ctx.rf_api_output_power_max - output_power_idx);
					while (TIM2_GetCounter() < (start_time + (ramp_step_duration_us * (output_power_idx + 1)))); // Wait until step duration is reached.
				}
				// Frequency shift is made in timer interrupt.
				while ((rf_api_ctx.rf_api_tim2_event_mask & (0b1 << TIM2_TIMINGS_ARRAY_CCR3_IDX)) == 0);
				// Ramp-up.
				start_time = TIM2_GetCounter();
				for (output_power_idx=0 ; output_power_idx<output_power_dynamic ; output_power_idx++) {
					SX1232_SetRfOutputPower(rf_api_ctx.rf_api_output_power_min + output_power_idx);
					while (TIM2_GetCounter() < (start_time + (ramp_step_duration_us * (output_power_idx + 1)))); // Wait until step duration is reached.
				}
			}
			else {
				rf_api_ctx.rf_api_phase_shift_required = 0;
			}
			// Wait the end of symbol period.
			while ((rf_api_ctx.rf_api_tim2_event_mask & (0b1 << TIM2_TIMINGS_ARRAY_ARR_IDX)) == 0);
		}
	}

	/* Last ramp down */
	start_time = TIM2_GetCounter();
	for (output_power_idx=0 ; output_power_idx<output_power_dynamic ; output_power_idx++) {
		SX1232_SetRfOutputPower(rf_api_ctx.rf_api_output_power_max - output_power_idx);
		while (TIM2_GetCounter() < (start_time + (first_last_ramp_step_duration_us * (output_power_idx + 1)))); // Wait until step duration is reached.
	}
	// Wait the end of symbol period.
	while ((rf_api_ctx.rf_api_tim2_event_mask & (0b1 << TIM2_TIMINGS_ARRAY_ARR_IDX)) == 0);

	/* Stop CW */
	TIM2_Stop();
	TIM2_Disable();
	SX1232_StopCw();

	/* Re-enable all interrupts */
#if (defined IM_RTC || defined CM_RTC)
	NVIC_EnableInterrupt(IT_RTC);
#endif
#ifdef ATM
	NVIC_EnableInterrupt(IT_USART2);
#ifdef RF_API_LOG_FRAME
	// Print frame on UART.
	USART2_SendString("sfx_frame = [");
	for (stream_byte_idx=0 ; stream_byte_idx<size ; stream_byte_idx++) {
		USART2_SendValue(sfx_frame[stream_byte_idx], USART_FORMAT_HEXADECIMAL, 1);
		if (stream_byte_idx < (size - 1)) {
			USART2_SendString(" ");
		}
	}
	USART2_SendString("]\n");
#endif
#endif
	return SFX_ERR_NONE;
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

	/* Start CW */
	SX1232_SetRfOutputPower(rf_api_ctx.rf_api_output_power_max);
	SX1232_StartCw();

	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_stop_continuous_transmission (void)
 * \brief Stop the current continuous transmisssion
 *
 * \retval SFX_ERR_NONE:                                 No error
 * \retval RF_ERR_API_STOP_CONTINUOUS_TRANSMISSION:      Continuous Transmission Stop error
 *******************************************************************/
sfx_u8 RF_API_stop_continuous_transmission (void) {

	/* Stop CW */
	SX1232_StopCw();

	return SFX_ERR_NONE;
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

	/* Program and save frequency */
	SX1232_SetRfFrequency(frequency);
	rf_api_ctx.rf_api_rf_frequency_hz = frequency;

	return SFX_ERR_NONE;
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
sfx_u8 RF_API_wait_frame(sfx_u8 *frame, sfx_s16 *rssi, sfx_rx_state_enum_t * state) {

	/* Start radio */
	SX1232_SetMode(SX1232_MODE_FSRX);
	LPTIM1_DelayMilliseconds(5); // Wait TS_FS=60µs typical.
	SX1232_SetMode(SX1232_MODE_RX);
	LPTIM1_DelayMilliseconds(5); // Wait TS_TR=120µs typical.

	/* Wait for external interrupt (payload ready on DIO0) */
	unsigned char rssi_retrieved = 0;
	unsigned int rx_window_start_time_seconds = TIM22_GetSeconds();
	(*state) = DL_PASSED;
	sfx_u8 sfx_err = SFX_ERR_NONE;
	while (GPIO_Read(GPIO_SX1232_DIO0) == 0) {
		// Get RSSI when preamble is found.
		if (((SX1232_GetIrqFlags() & 0x0200) != 0) && (rssi_retrieved == 0)) {
			(*rssi) = (sfx_s16) ((-1) * SX1232_GetRssi());
			rssi_retrieved = 1;
		}
		// Exit if 25 seconds ellapsed.
		if (TIM22_GetSeconds() > (rx_window_start_time_seconds + RF_API_DOWNLINK_TIMEOUT_SECONDS)) {
			(*state) = DL_TIMEOUT;
			sfx_err = RF_ERR_API_WAIT_FRAME;
			break;
		}
	}

	/* Read FIFO if data was retrieved */
	if ((*state) == DL_PASSED) {
		SX1232_ReadFifo(frame, RF_API_DOWNLINK_FRAME_LENGTH_BYTES);
	}

	return sfx_err;
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
	return SFX_ERR_NONE;
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
	return SFX_ERR_NONE;
}
