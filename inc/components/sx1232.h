/*
 * sx1232.h
 *
 *  Created on: 20 june 2018
 *      Author: Ludo
 */

#ifndef SX1232_H
#define SX1232_H

/*** SX1232 macros ***/

// Frequency range.
#define SX1232_RF_FREQUENCY_HZ_MIN				862000000
#define SX1232_RF_FREQUENCY_HZ_MAX				1020000000

// TCXO and output clock frequency (DIO5).
#define SX1232_FXOSC_HZ							32000000
#define SX1232_CLKOUT_PRESCALER					2
#define SX1232_CLKOUT_FREQUENCY_KHZ				((SX1232_FXOSC_HZ) / (SX1232_CLKOUT_PRESCALER * 1000))

// Output power ranges.
#define SX1232_OUTPUT_POWER_RFO_MIN				0
#define SX1232_OUTPUT_POWER_RFO_MAX				17
#define SX1232_OUTPUT_POWER_PABOOST_MIN			2
#define SX1232_OUTPUT_POWER_PABOOST_MAX			20

// SX1232 RXBW exponent maximum value.
#define SX1232_RXBW_EXPONENT_MAX				7

// RSSI offset due to internal and external LNA (calibration).
#define SX1232_RSSI_OFFSET_DB					24

/*** SX1232 structures ***/

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
void SX1232_tcxo(unsigned char tcxo_enable);
// Common settings.
void SX1232_set_oscillator(SX1232_oscillator_t oscillator);
void SX1232_set_mode(SX1232_mode_t mode);
void SX1232_set_modulation(SX1232_modulation_t modulation, SX1232_modulation_shaping_t modulation_shaping);
void SX1232_set_rf_frequency(unsigned int rf_frequency_hz);
void SX1232_enable_fast_frequency_hopping(void);
unsigned int SX1232_get_rf_frequency(void);
void SX1232_set_fsk_deviation(unsigned short fsk_deviation_hz);
void SX1232_set_bitrate(unsigned int bit_rate_bps);
void SX1232_set_data_mode(SX1232_data_mode_t data_mode);
void SX1232_set_dio_mapping(unsigned char diox, unsigned char diox_mapping);
unsigned short SX1232_get_irq_flags(void);
// TX functions.
void SX1232_set_rf_output_pin(SX1232_rf_output_pin_t rf_output_pin);
void SX1232_set_rf_output_power(unsigned char rf_output_power_dbm);
void SX1232_enable_low_phase_noise_pll(void);
void SX1232_start_cw(void);
void SX1232_stop_cw(void);
// RX functions.
void SX1232_set_rx_bandwidth(SX1232_rxbw_mantissa_t rxbw_mantissa, unsigned char rxbw_exponent);
void SX1232_enable_lna_boost(unsigned char lna_boost_enable);
void SX1232_set_preamble_detector(unsigned char preamble_length_bytes, unsigned char preamble_polarity);
void SX1232_set_sync_word(unsigned char* sync_word, unsigned char sync_word_length_bytes);
void SX1232_set_data_length(unsigned char data_length_bytes);
void SX1232_configure_rssi(signed char rssi_offset, SX1232_rssi_sampling_t rssi_sampling);
signed short SX1232_get_rssi(void);
void SX1232_read_fifo(unsigned char* rx_data, unsigned char rx_data_length);

#endif /* SX1232_H */
