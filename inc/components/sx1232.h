/*
 * sx1232.h
 *
 *  Created on: 20 jun. 2018
 *      Author: Ludo
 */

#ifndef __SX1232_H__
#define __SX1232_H__

#include "lptim.h"
#include "spi.h"
#include "types.h"

/*** SX1232 macros ***/

#define SX1232_STATE_SWITCH_DELAY_MS	5
#define SX1232_OSCILLATOR_DELAY_MS		5
#define SX1232_START_TX_DELAY_MS		(2 * SX1232_STATE_SWITCH_DELAY_MS)
#define SX1232_START_RX_DELAY_MS		(2 * SX1232_STATE_SWITCH_DELAY_MS)

/*** SX1232 structures ***/

/*!******************************************************************
 * \enum SX1232_status_t
 * \brief SX1232 driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	SX1232_SUCCESS = 0,
	SX1232_ERROR_NULL_PARAMETER,
	SX1232_ERROR_REGISTER_ADDRESS,
	SX1232_ERROR_OSCILLATOR,
	SX1232_ERROR_MODE,
	SX1232_ERROR_IRQ_INDEX,
	SX1232_ERROR_MODULATION,
	SX1232_ERROR_MODULATION_SHAPING,
	SX1232_ERROR_RF_FREQUENCY_OVERFLOW,
	SX1232_ERROR_RF_FREQUENCY_UNDERFLOW,
	SX1232_ERROR_FSK_DEVIATION,
	SX1232_ERROR_BIT_RATE_OVERFLOW,
	SX1232_ERROR_BIT_RATE_UNDERFLOW,
	SX1232_ERROR_DATA_MODE,
	SX1232_ERROR_DIO,
	SX1232_ERROR_DIO_MAPPING,
	SX1232_ERROR_RF_OUTPUT_PIN,
	SX1232_ERROR_RF_OUTPUT_POWER_OVERFLOW,
	SX1232_ERROR_RF_OUTPUT_POWER_UNDERFLOW,
	SX1232_ERROR_RXBW_MANTISSA,
	SX1232_ERROR_RXBW_EXPONENT,
	SX1232_ERROR_PREAMBLE_LENGTH,
	SX1232_ERROR_SYNC_WORD_LENGTH,
	SX1232_ERROR_RSSI_SAMPLING,
	SX1232_ERROR_FIFO_LENGTH,
	// Low level drivers errors.
	SX1232_ERROR_BASE_SPI1 = 0x0100,
	SX1232_ERROR_BASE_LPTIM1 = (SX1232_ERROR_BASE_SPI1 + SPI_ERROR_BASE_LAST),
	// Last base value.
	SX1232_ERROR_BASE_LAST = (SX1232_ERROR_BASE_LPTIM1 + LPTIM_ERROR_BASE_LAST)
} SX1232_status_t;

/*!******************************************************************
 * \enum SX1232_oscillator_t
 * \brief SX1232 oscillator types.
 *******************************************************************/
typedef enum {
	SX1232_OSCILLATOR_QUARTZ,
	SX1232_OSCILLATOR_TCXO,
	SX1232_OSCILLATOR_LAST
} SX1232_oscillator_t;

/*!******************************************************************
 * \enum SX1232_mode_t
 * \brief SX1232 transceiver modes.
 *******************************************************************/
typedef enum {
	SX1232_MODE_SLEEP,
	SX1232_MODE_STANDBY,
	SX1232_MODE_FSTX,
	SX1232_MODE_TX,
	SX1232_MODE_FSRX,
	SX1232_MODE_RX,
	SX1232_MODE_LAST
} SX1232_mode_t;

/*!******************************************************************
 * \enum SX1232_mode_t
 * \brief SX1232 GPIOs list
 *******************************************************************/
typedef enum {
	SX1232_DIO0 = 0,
	SX1232_DIO1,
	SX1232_DIO2,
	SX1232_DIO3,
	SX1232_DIO4,
	SX1232_DIO5,
	SX1232_DIO_LAST
} SX1232_dio_t;

/*!******************************************************************
 * \enum SX1232_irq_index_t
 * \brief SX1232 internal interrupts list.
 *******************************************************************/
typedef enum {
	SX1232_IRQ_INDEX_LOW_BAT = 0,
	SX1232_IRQ_INDEX_CRC_OK,
	SX1232_IRQ_INDEX_PAYLOAD_READY,
	SX1232_IRQ_INDEX_PACKET_SENT,
	SX1232_IRQ_INDEX_FIFO_OVERRUN,
	SX1232_IRQ_INDEX_FIFO_LEVEL,
	SX1232_IRQ_INDEX_FIFO_EMPTY,
	SX1232_IRQ_INDEX_FIFO_FULL,
	SX1232_IRQ_INDEX_SYNC_ADDRESS_MATCH,
	SX1232_IRQ_INDEX_PREAMBLE_DETECT,
	SX1232_IRQ_INDEX_TIMEOUT,
	SX1232_IRQ_INDEX_RSSI,
	SX1232_IRQ_INDEX_PLL_LOCK,
	SX1232_IRQ_INDEX_TX_READY,
	SX1232_IRQ_INDEX_RX_READY,
	SX1232_IRQ_INDEX_MODE_READY,
	SX1232_IRQ_INDEX_LAST
} SX1232_irq_index_t;

/*!******************************************************************
 * \enum SX1232_dio_mapping_t
 * \brief SX1232 GPIOs functions mapping.
 *******************************************************************/
typedef enum {
	SX1232_DIO_MAPPING0 = 0,
	SX1232_DIO_MAPPING1,
	SX1232_DIO_MAPPING2,
	SX1232_DIO_MAPPING3,
	SX1232_DIO_MAPPING_LAST
} SX1232_dio_mapping_t;

/*!******************************************************************
 * \enum SX1232_rf_output_pin_t
 * \brief SX1232 RF output pin selection.
 *******************************************************************/
typedef enum {
	SX1232_RF_OUTPUT_PIN_RFO,
	SX1232_RF_OUTPUT_PIN_PABOOST,
	SX1232_RF_OUTPUT_PIN_LAST
} SX1232_rf_output_pin_t;

/*!******************************************************************
 * \enum SX1232_modulation_t
 * \brief SX1232 modulations list.
 *******************************************************************/
typedef enum {
	SX1232_MODULATION_FSK,
	SX1232_MODULATION_OOK,
	SX1232_MODULATION_LAST
} SX1232_modulation_t;

/*!******************************************************************
 * \enum SX1232_modulation_shaping_t
 * \brief SX1232 modulations shaping list.
 *******************************************************************/
typedef enum {
	SX1232_MODULATION_SHAPING_NONE,
	SX1232_MODULATION_SHAPING_FSK_BT_1,
	SX1232_MODULATION_SHAPING_FSK_BT_05,
	SX1232_MODULATION_SHAPING_FSK_BT_03,
	SX1232_MODULATION_SHAPING_OOK_BITRATE,
	SX1232_MODULATION_SHAPING_OOK_TWO_BITRATE,
	SX1232_MODULATION_SHAPING_LAST
} SX1232_modulation_shaping_t;

/*!******************************************************************
 * \enum SX1232_data_mode_t
 * \brief SX1232 data modes.
 *******************************************************************/
typedef enum {
	SX1232_DATA_MODE_PACKET,
	SX1232_DATA_MODE_CONTINUOUS,
	SX1232_DATA_MODE_LAST
} SX1232_data_mode_t;

/*!******************************************************************
 * \enum SX1232_rxbw_mantissa_t
 * \brief SX1232 RX bandwidth mantissa values.
 *******************************************************************/
typedef enum {
	SX1232_RXBW_MANTISSA_16,
	SX1232_RXBW_MANTISSA_20,
	SX1232_RXBW_MANTISSA_24,
	SX1232_RXBW_MANTISSA_LAST
} SX1232_rxbw_mantissa_t;

/*!******************************************************************
 * \enum SX1232_rxbw_exponent_t
 * \brief SX1232 RX bandwidth exponent values.
 *******************************************************************/
typedef enum {
	SX1232_RXBW_EXPONENT_0 = 0,
	SX1232_RXBW_EXPONENT_1,
	SX1232_RXBW_EXPONENT_2,
	SX1232_RXBW_EXPONENT_3,
	SX1232_RXBW_EXPONENT_4,
	SX1232_RXBW_EXPONENT_5,
	SX1232_RXBW_EXPONENT_6,
	SX1232_RXBW_EXPONENT_7,
	SX1232_RXBW_EXPONENT_LAST
} SX1232_rxbw_exponent_t;

/*!******************************************************************
 * \enum SX1232_rssi_sampling_t
 * \brief SX1232 RSSI sampling ratios.
 *******************************************************************/
typedef enum {
	SX1232_RSSI_SAMPLING_2,
	SX1232_RSSI_SAMPLING_4,
	SX1232_RSSI_SAMPLING_8,
	SX1232_RSSI_SAMPLING_16,
	SX1232_RSSI_SAMPLING_32,
	SX1232_RSSI_SAMPLING_64,
	SX1232_RSSI_SAMPLING_128,
	SX1232_RSSI_SAMPLING_256,
	SX1232_RSSI_SAMPLING_LAST
} SX1232_rssi_sampling_t;

/*** SX1232 functions ***/

/*!******************************************************************
 * \fn void SX1232_init(void)
 * \brief Init SX1232 driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void SX1232_init(void);

/*!******************************************************************
 * \fn void SX1232_de_init_init(void)
 * \brief Release SX1232 driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void SX1232_de_init(void);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_set_oscillator(SX1232_oscillator_t oscillator)
 * \brief Set SX1232 oscillator type.
 * \param[in]  	oscillator: Type of oscillator attached to the SX1232.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_set_oscillator(SX1232_oscillator_t oscillator);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_set_mode(SX1232_mode_t mode)
 * \brief Set SX1232 state.
 * \param[in]  	mode: New mode to switch to.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_set_mode(SX1232_mode_t mode);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_set_modulation(SX1232_modulation_t modulation, SX1232_modulation_shaping_t modulation_shaping)
 * \brief Configure SX1232 modulation scheme.
 * \param[in]  	modulation: Modulation to set.
 * \param[in]	modulation_shaping: Modulation shaping to set.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_set_modulation(SX1232_modulation_t modulation, SX1232_modulation_shaping_t modulation_shaping);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_set_rf_frequency(uint32_t rf_frequency_hz)
 * \brief Set SX1232 RF center frequency.
 * \param[in]  	rf_frequency_hz: Center frequency to set in Hz.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_set_rf_frequency(uint32_t rf_frequency_hz);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_enable_low_phase_noise_pll(void)
 * \brief Enable SX1232 low phase noise mode of PLL.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_enable_low_phase_noise_pll(void);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_set_fsk_deviation(uint32_t fsk_deviation_hz)
 * \brief Set SX1232 FSK deviation.
 * \param[in]  	fsk_deviation_hz: FSK deviation in Hz.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_set_fsk_deviation(uint32_t fsk_deviation_hz);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_set_bitrate(uint32_t bit_rate_bps)
 * \brief Set SX1232 bit rate.
 * \param[in]  	bit_rate_bps: Bit rate in bps.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_set_bitrate(uint32_t bit_rate_bps);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_set_data_mode(SX1232_data_mode_t data_mode)
 * \brief Set SX1232 data mode.
 * \param[in]  	data_mode: Data mode to use.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_set_data_mode(SX1232_data_mode_t data_mode);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_set_data_size(uint8_t data_size_bytes)
 * \brief Set SX1232 packet payload size.
 * \param[in]  	data_size_bytes: Data size in bytes.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_set_data_size(uint8_t data_size_bytes);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_set_dio_mapping(SX1232_dio_t dio, SX1232_dio_mapping_t dio_mapping)
 * \brief Configure SX1232 DIO.
 * \param[in]  	dio: GPIO to configure.
 * \param[in]	dio_mapping: Function to attach to GPIO.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_set_dio_mapping(SX1232_dio_t dio, SX1232_dio_mapping_t dio_mapping);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_get_irq_flag(SX1232_irq_index_t irq_index, uint8_t* irq_flag)
 * \brief Read SX1232 internal interrupt status.
 * \param[in]  	irq_index: Interrupt index.
 * \param[out] 	irq_flag: Pointer to bit that will contain interrupt status.
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_get_irq_flag(SX1232_irq_index_t irq_index, uint8_t* irq_flag);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_start_tx(void)
 * \brief Start transmission.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_start_tx(void);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_set_rf_output_pin(SX1232_rf_output_pin_t rf_output_pin)
 * \brief Set SX1232 RF output pin.
 * \param[in]  	rf_output_pin: RF output pin to use for TX.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_set_rf_output_pin(SX1232_rf_output_pin_t rf_output_pin);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_set_rf_output_power(int8_t rf_output_power_dbm)
 * \brief Set SX1232 RF output power.
 * \param[in]  	rf_output_power_dbm: RF output power in dBm.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_set_rf_output_power(int8_t rf_output_power_dbm);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_enable_manual_pa_control(void)
 * \brief Enable manual control of the PA amplitude.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_enable_manual_pa_control(void);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_set_pa_power_value(uint8_t pa_power_value)
 * \brief Optimized function to set SX1232 PA power value.
 * \param[in]  	pa_power_value: PA power register value.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void SX1232_set_pa_power_value(uint8_t pa_power_value);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_start_rx(void)
 * \brief Start reception.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_start_rx(void);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_set_rx_bandwidth(SX1232_rxbw_mantissa_t rxbw_mantissa, SX1232_rxbw_exponent_t rxbw_exponent)
 * \brief Set SX1232 received bandwidth.
 * \param[in]  	rxbw_mantissa: Mantissa value of the RX bandwidth.
 * \param[in]	rxbw_exponent: Exponent value of the RX bandwidth.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_set_rx_bandwidth(SX1232_rxbw_mantissa_t rxbw_mantissa, SX1232_rxbw_exponent_t rxbw_exponent);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_enable_lna_boost(uint8_t lna_boost_enable)
 * \brief Configure SX1232 internal LNA.
 * \param[in]  	lna_boost_enable: Enable internal LNA if non zero.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_enable_lna_boost(uint8_t lna_boost_enable);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_set_preamble_detector(uint8_t preamble_size_bytes, uint8_t preamble_polarity)
 * \brief Configure SX1232 RX preamble detector.
 * \param[in]  	preamble_size_bytes: Length of the preamble in bytes.
 * \param[in]	preamble_polarity: Preamble polarity.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_set_preamble_detector(uint8_t preamble_size_bytes, uint8_t preamble_polarity);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_set_sync_word(uint8_t* sync_word, uint8_t sync_word_size_bytes)
 * \brief Configure SX1232 RX synchronization word.
 * \param[in]  	sync_word: Synchronization word byte array.
 * \param[in]	sync_word_size_bytes: Length of the synchronization word.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_set_sync_word(uint8_t* sync_word, uint8_t sync_word_size_bytes);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_configure_rssi(SX1232_rssi_sampling_t rssi_sampling)
 * \brief Configure SX1232 RX RSSI sampling.
 * \param[in]  	rssi_sampling: RSSI sampling to use.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_configure_rssi(SX1232_rssi_sampling_t rssi_sampling);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_get_rssi(int16_t* rssi_dbm)
 * \brief Get SX1232 RX RSSI.
 * \param[in]  	none
 * \param[out] 	rssi_dbm: Pointer to signed 16-bits value that will contain the RSSI in dBm.
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_get_rssi(int16_t* rssi_dbm);

/*!******************************************************************
 * \fn SX1232_status_t SX1232_read_fifo(uint8_t* fifo_data, uint8_t fifo_data_size)
 * \brief Read SX1232 FIFO.
 * \param[in]  	fifo_data_size: Number of bytes to read.
 * \param[out] 	fifo_data: Byte array that will contain the RX FIFO bytes.
 * \retval		Function execution status.
 *******************************************************************/
SX1232_status_t SX1232_read_fifo(uint8_t* fifo_data, uint8_t fifo_data_size);

/*******************************************************************/
#define SX1232_exit_error(error_base) { if (sx1232_status != SX1232_SUCCESS) { status = (error_base + sx1232_status); goto errors; } }

/*******************************************************************/
#define SX1232_stack_error(void) { if (sx1232_status != SX1232_SUCCESS) { ERROR_stack_add(ERROR_BASE_SX1232 + sx1232_status); } }

/*******************************************************************/
#define SX1232_stack_exit_error(error_code) { if (sx1232_status != SX1232_SUCCESS) { ERROR_stack_add(ERROR_BASE_SX1232 + sx1232_status); status = error_code; goto errors; } }

#endif /* __SX1232_H__ */
