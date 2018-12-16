/*
 * rf_api.c
 *
 *  Created on: 18 juin 2018
 *      Author: Ludovic
 */

#include "rf_api.h"

#include "lptim.h"
#include "nvic.h"
#include "sigfox_api.h"
#include "sigfox_types.h"
#include "sky13317.h"
#include "spi.h"
#include "sx1232.h"

/*** RF API local structures ***/

// Sigfox uplink modulation parameters.
typedef struct {
	// Uplink message frequency.
	unsigned int uplink_frequency_hz;
	// Modulation parameters.
	unsigned short first_ramp_up_duration_us;
	unsigned short last_ramp_down_duration_us;
	unsigned short symbol_duration_us;
	unsigned short ramp_duration_us;
	unsigned short frequency_shift_duration_us;
	unsigned int frequency_shift_hz;
	// Output power range.
	signed char output_power_min;
	signed char output_power_max;
} RF_API_Context;

/*** RF API local global variables ***/

RF_API_Context rf_api_ctx;

/*** RF API local functions ***/

/* UPDATE PARAMETERS ACCORDING TO MODULATION TYPE.
 * @param modulation:	Modulation type asked by Sigfox library.
 * @return:				None.
 */
void RF_API_SetTxModulationParameters(sfx_modulation_type_t modulation) {
	switch (modulation) {
	case SFX_DBPSK_100BPS:
		// 100 bps timings.
		rf_api_ctx.first_ramp_up_duration_us = 10000;
		rf_api_ctx.last_ramp_down_duration_us = 10000;
		rf_api_ctx.symbol_duration_us = 10000;
		rf_api_ctx.ramp_duration_us = 200;
		rf_api_ctx.frequency_shift_duration_us = 400;
		rf_api_ctx.frequency_shift_hz = 2500;
		break;
	case SFX_DBPSK_600BPS:
		// 600 bps timings.
		rf_api_ctx.first_ramp_up_duration_us = 1667;
		rf_api_ctx.last_ramp_down_duration_us = 1667;
		rf_api_ctx.symbol_duration_us = 1667;
		rf_api_ctx.ramp_duration_us = 200;
		rf_api_ctx.frequency_shift_duration_us = 400;
		rf_api_ctx.frequency_shift_hz = 2500;
		break;
	default:
		break;
	}
}

/* SELECT TX RF PATH ACCORDING TO MODULATION TYPE.
 * @param modulation:	Modulation type asked by Sigfox library.
 * @return:				None.
 */
void RF_API_SetTxPath(sfx_modulation_type_t modulation) {
	switch (modulation) {

	case SFX_DBPSK_100BPS:
		// Assume 100bps corresponds to ETSI configuration (14dBm on PABOOST pin).
		SX1232_SelectRfOutputPin(SX1232_RF_OUTPUT_PIN_PABOOST);
		SKY13317_SetChannel(SKY13317_CHANNEL_RF1);
		rf_api_ctx.output_power_min = SX1232_OUTPUT_POWER_PABOOST_MIN;
		rf_api_ctx.output_power_max = SX1232_OUTPUT_POWER_PABOOST_MAX;
		break;

	case SFX_DBPSK_600BPS:
		// Assume 600bps corresponds to FCC configuration (22dBm with PA on RFO pin).
		SX1232_SelectRfOutputPin(SX1232_RF_OUTPUT_PIN_RFO);
		SKY13317_SetChannel(SKY13317_CHANNEL_RF3);
		rf_api_ctx.output_power_min = SX1232_OUTPUT_POWER_RFO_MIN;
		rf_api_ctx.output_power_max = SX1232_OUTPUT_POWER_RFO_MAX;
		break;

	default:
		break;
	}
}

/* SELECT TX RF PATH ACCORDING TO MODULATION TYPE.
 * @param modulation:	Modulation type asked by Sigfox library.
 * @return:				None.
 */
void RF_API_SetRxPath(void) {
	// Activate LNA.
	SKY13317_SetChannel(SKY13317_CHANNEL_RF2);
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
	SPI_PowerOn();
	SX1232_Init();

	/* Configure transceiver */
	switch (rf_mode) {

	case SFX_RF_MODE_TX:
		// Prepare transceiver for TX operation.
		SX1232_SetModulation(SX1232_MODULATION_OOK); // Modulation OOK for uplnik DBPSK implementation.
		SX1232_SetMode(SX1232_MODE_FSTX); // FSTX mode prior to TX mode to configure frequency.
		break;

	case SFX_RF_MODE_RX:
		// Prepare transceiver for RX operation.
		SX1232_SetModulation(SX1232_MODULATION_FSK); // Modulation FSK for downlink GFSK.
		SX1232_SetMode(SX1232_MODE_FSRX); // FSRX mode prior to RX mode to configure frequency.
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

	/* Switch RF off */
	SPI_PowerOff();

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
#if (defined IM_RTC || defined CM_RTC)
	NVIC_DisableInterrupt(IT_RTC);
#endif
#ifdef CM_RTC
	NVIC_DisableInterrupt(IT_EXTI4_15);
	NVIC_DisableInterrupt(IT_TIM21);
#endif
#ifdef ATM
	NVIC_DisableInterrupt(IT_USART2);
#endif
	NVIC_DisableInterrupt(IT_LPUART1);

	/* Set modulation and RF path according to modulation type */
	RF_API_SetTxModulationParameters(type);
	RF_API_SetTxPath(type);

	/* Init all variables before transmission */
	unsigned short idle_duration_us = (rf_api_ctx.symbol_duration_us - (2 * rf_api_ctx.ramp_duration_us) - (rf_api_ctx.frequency_shift_duration_us)) / (2);
	unsigned char output_power_dynamic = (rf_api_ctx.output_power_max - rf_api_ctx.output_power_min) + 1;
	unsigned short first_ramp_up_step_duration_us = rf_api_ctx.first_ramp_up_duration_us / output_power_dynamic;
	unsigned short last_ramp_down_step_duration_us = rf_api_ctx.last_ramp_down_duration_us / output_power_dynamic;
	unsigned short ramp_step_duration_us = rf_api_ctx.ramp_duration_us / output_power_dynamic;
	unsigned char frequency_shift_direction = 0;
	unsigned short start_time = 0;
	unsigned char output_power_idx = 0;
	unsigned char stream_byte_idx = 0;
	unsigned char stream_bit_idx = 0;

	/* Start CW */
	SX1232_SetRfFrequency(rf_api_ctx.uplink_frequency_hz);
	SX1232_SetRfOutputPower(rf_api_ctx.output_power_min);
	SX1232_SetMode(SX1232_MODE_TX);
	SX1232_StartContinuousTransmission();

	/* First ramp-up */
	// Start timer with maximum value (since ARRM flag is not used for this sequence).
	LPTIM1_Start(0xFFFF);
	for (output_power_idx=0 ; output_power_idx<output_power_dynamic ; output_power_idx++) {
		SX1232_SetRfOutputPower(rf_api_ctx.output_power_min + output_power_idx); // Update output power.
		while (LPTIM1_GetMicroseconds() < (first_ramp_up_step_duration_us * (output_power_idx + 1))); // Wait until step duration is reached.
	}
	LPTIM1_Stop();

	/* Data transmission */
	// Start timer with symbol period.
	LPTIM1_Start(rf_api_ctx.symbol_duration_us);
	// Byte loop.
	for (stream_byte_idx=0 ; stream_byte_idx<size ; stream_byte_idx++) {
		// Bit loop.
		for (stream_bit_idx=0 ; stream_bit_idx<8 ; stream_bit_idx++) {
			// Check bit.
			if ((stream[stream_byte_idx] & (0b1 << (7-stream_bit_idx))) == 0) {
				// Bit is '0' -> phase shift is required.
				// First idle period.
				while (LPTIM1_GetMicroseconds() < idle_duration_us);
				// Ramp down.
				start_time = LPTIM1_GetMicroseconds();
				for (output_power_idx=0 ; output_power_idx<output_power_dynamic ; output_power_idx++) {
					SX1232_SetRfOutputPower(rf_api_ctx.output_power_max - output_power_idx);
					while (LPTIM1_GetMicroseconds() < (start_time + (ramp_step_duration_us * (output_power_idx + 1)))); // Wait until step duration is reached.
				}
				// Frequency shift (alternatively lower and higher).
				start_time = LPTIM1_GetMicroseconds();
				if (frequency_shift_direction == 0) {
					// Decrease frequency.
					SX1232_SetRfFrequency(rf_api_ctx.uplink_frequency_hz - rf_api_ctx.frequency_shift_hz);
					frequency_shift_direction = 1;
				}
				else {
					// Increase frequency.
					SX1232_SetRfFrequency(rf_api_ctx.uplink_frequency_hz + rf_api_ctx.frequency_shift_hz);
					frequency_shift_direction = 0;
				}
				while (LPTIM1_GetMicroseconds() < (start_time + rf_api_ctx.frequency_shift_duration_us));
				// Ramp up.
				start_time = LPTIM1_GetMicroseconds();
				for (output_power_idx=0 ; output_power_idx<output_power_dynamic ; output_power_idx++) {
					SX1232_SetRfOutputPower(rf_api_ctx.output_power_min + output_power_idx);
					while (LPTIM1_GetMicroseconds() < (start_time + (ramp_step_duration_us * (output_power_idx + 1)))); // Wait until step duration is reached.
				}
			}
			// Wait the end of symbol period.
			while (LPTIM1_GetArrmFlag() == 0);
			LPTIM1_ClearArrmFlag();
		}
	}
	LPTIM1_Stop();

	/* Last ramp down */
	// Start timer with maximum value (since ARRM flag is not used for this sequence).
	LPTIM1_Start(0xFFFF);
	for (output_power_idx=0 ; output_power_idx<output_power_dynamic ; output_power_idx++) {
		// Update output power.
		SX1232_SetRfOutputPower(rf_api_ctx.output_power_max - output_power_idx);
		// Wait until step duration is reached.
		while (LPTIM1_GetMicroseconds() < (last_ramp_down_step_duration_us * (output_power_idx + 1)));
	}
	LPTIM1_Stop();

	/* Stop CW */
	SX1232_StopContinuousTransmission();
	SX1232_SetMode(SX1232_MODE_SLEEP);

	/* Re-enable all interrupts */
#if (defined IM_RTC || defined CM_RTC)
	NVIC_EnableInterrupt(IT_RTC);
#endif
#ifdef CM_RTC
	NVIC_EnableInterrupt(IT_EXTI4_15);
	NVIC_EnableInterrupt(IT_TIM21);
#endif
#ifdef ATM
	NVIC_EnableInterrupt(IT_USART2);
#endif
	NVIC_EnableInterrupt(IT_LPUART1);

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
	SX1232_SetRfFrequency(rf_api_ctx.uplink_frequency_hz);
	SX1232_SetRfOutputPower(rf_api_ctx.output_power_max);
	SX1232_SetMode(SX1232_MODE_TX);
	SX1232_StartContinuousTransmission();

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
	SX1232_StopContinuousTransmission();
	SX1232_SetMode(SX1232_MODE_SLEEP);

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

	/* Save frequency for uplink modulation implementation */
	rf_api_ctx.uplink_frequency_hz = frequency;

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
	return SFX_ERR_NONE;
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
