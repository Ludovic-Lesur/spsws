/*
 * sx1232.h
 *
 *  Created on: 20 june 2018
 *      Author: Ludo
 */

#ifndef SX1232_H
#define SX1232_H

#include "spi.h"

/*** SX1232 structures ***/

typedef enum {
	SX1232_SUCCESS = 0,
	SX1232_ERROR_ADDRESS,
	SX1232_ERROR_OSCILLATOR,
	SX1232_ERROR_MODE,
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
	SX1232_ERROR_BASE_SPI = 0x0100,
	SX1232_ERROR_BASE_LPTIM = (SX1232_ERROR_BASE_SPI + SPI_ERROR_BASE_LAST),
	SX1232_ERROR_BASE_LAST = (SX1232_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST)
} SX1232_status_t;

// Oscillator configuration.
typedef enum {
	SX1232_OSCILLATOR_QUARTZ,
	SX1232_OSCILLATOR_TCXO,
	SX1232_OSCILLATOR_LAST
} SX1232_oscillator_t;

// Transceiver modes.
typedef enum {
	SX1232_MODE_SLEEP,
	SX1232_MODE_STANDBY,
	SX1232_MODE_FSTX,
	SX1232_MODE_TX,
	SX1232_MODE_FSRX,
	SX1232_MODE_RX,
	SX1232_MODE_LAST
} SX1232_mode_t;

typedef enum {
	SX1232_DIO0 = 0,
	SX1232_DIO1,
	SX1232_DIO2,
	SX1232_DIO3,
	SX1232_DIO4,
	SX1232_DIO5,
	SX1232_DIO_LAST
} SX1232_dio_t;

typedef enum {
	SX1232_DIO_MAPPING0 = 0,
	SX1232_DIO_MAPPING1,
	SX1232_DIO_MAPPING2,
	SX1232_DIO_MAPPING3,
	SX1232_DIO_MAPPING_LAST
} SX1232_dio_mapping_t;

// Modulation.
typedef enum {
	SX1232_MODULATION_FSK,
	SX1232_MODULATION_OOK,
	SX1232_MODULATION_LAST
} SX1232_modulation_t;

// Modulation shaping.
typedef enum {
	SX1232_MODULATION_SHAPING_NONE,
	SX1232_MODULATION_SHAPING_FSK_BT_1,
	SX1232_MODULATION_SHAPING_FSK_BT_05,
	SX1232_MODULATION_SHAPING_FSK_BT_03,
	SX1232_MODULATION_SHAPING_OOK_BITRATE,
	SX1232_MODULATION_SHAPING_OOK_TWO_BITRATE,
	SX1232_MODULATION_SHAPING_LAST
} SX1232_modulation_shaping_t;

// RF output pin.
typedef enum {
	SX1232_RF_OUTPUT_PIN_RFO,
	SX1232_RF_OUTPUT_PIN_PABOOST,
	SX1232_RF_OUTPUT_PIN_LAST
} SX1232_rf_output_pin_t;

// Data mode.
typedef enum {
	SX1232_DATA_MODE_PACKET,
	SX1232_DATA_MODE_CONTINUOUS,
	SX1232_DATA_MODE_LAST
} SX1232_data_mode_t;

// RX bandwidth mantissa.
typedef enum {
	SX1232_RXBW_MANTISSA_16,
	SX1232_RXBW_MANTISSA_20,
	SX1232_RXBW_MANTISSA_24,
	SX1232_RXBW_MANTISSA_LAST
} SX1232_rxbw_mantissa_t;

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

// RSSI sampling.
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

void SX1232_init(void);
void SX1232_disable(void);
SX1232_status_t SX1232_tcxo(unsigned char tcxo_enable);
// Common settings.
SX1232_status_t SX1232_set_oscillator(SX1232_oscillator_t oscillator);
SX1232_status_t SX1232_set_mode(SX1232_mode_t mode);
SX1232_status_t SX1232_set_modulation(SX1232_modulation_t modulation, SX1232_modulation_shaping_t modulation_shaping);
SX1232_status_t SX1232_set_rf_frequency(unsigned int rf_frequency_hz);
SX1232_status_t SX1232_get_rf_frequency(unsigned int* rf_frequency_hz);
SX1232_status_t SX1232_enable_fast_frequency_hopping(void);
SX1232_status_t SX1232_set_fsk_deviation(unsigned int fsk_deviation_hz);
SX1232_status_t SX1232_set_bitrate(unsigned int bit_rate_bps);
SX1232_status_t SX1232_set_data_mode(SX1232_data_mode_t data_mode);
SX1232_status_t SX1232_set_dio_mapping(SX1232_dio_t dio, SX1232_dio_mapping_t dio_mapping);
SX1232_status_t SX1232_get_irq_flags(unsigned int* irq_flags);
// TX functions.
SX1232_status_t SX1232_set_rf_output_pin(SX1232_rf_output_pin_t rf_output_pin);
SX1232_status_t SX1232_set_rf_output_power(unsigned char rf_output_power_dbm);
SX1232_status_t SX1232_enable_low_phase_noise_pll(void);
SX1232_status_t SX1232_start_cw(void);
SX1232_status_t SX1232_stop_cw(void);
// RX functions.
SX1232_status_t SX1232_set_rx_bandwidth(SX1232_rxbw_mantissa_t rxbw_mantissa, SX1232_rxbw_exponent_t rxbw_exponent);
SX1232_status_t SX1232_enable_lna_boost(unsigned char lna_boost_enable);
SX1232_status_t SX1232_set_preamble_detector(unsigned char preamble_length_bytes, unsigned char preamble_polarity);
SX1232_status_t SX1232_set_sync_word(unsigned char* sync_word, unsigned char sync_word_length_bytes);
SX1232_status_t SX1232_set_data_length(unsigned char data_length_bytes);
SX1232_status_t SX1232_configure_rssi(SX1232_rssi_sampling_t rssi_sampling);
SX1232_status_t SX1232_get_rssi(signed short* rssi_dbm);
SX1232_status_t SX1232_read_fifo(unsigned char* fifo_data, unsigned char fifo_data_length);

#define SX1232_status_check(error_base) { if (sx1232_status != SX1232_SUCCESS) { status = error_base + sx1232_status; goto errors; }}
#define SX1232_error_check() { ERROR_status_check(sx1232_status, SX1232_SUCCESS, ERROR_BASE_SX1232); }

#endif /* SX1232_H */
