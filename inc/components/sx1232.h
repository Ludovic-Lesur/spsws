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
	SX1232_OSCILLATOR_TCXO
} SX1232_Oscillator;

// Transceiver modes.
typedef enum {
	SX1232_MODE_SLEEP,
	SX1232_MODE_STANDBY,
	SX1232_MODE_FSTX,
	SX1232_MODE_TX,
	SX1232_MODE_FSRX,
	SX1232_MODE_RX
} SX1232_Mode;

// Modulation.
typedef enum {
	SX1232_MODULATION_FSK,
	SX1232_MODULATION_OOK
} SX1232_Modulation;

// Modulation shaping.
typedef enum {
	SX1232_MODULATION_SHAPING_NONE,
	SX1232_MODULATION_SHAPING_FSK_BT_1,
	SX1232_MODULATION_SHAPING_FSK_BT_05,
	SX1232_MODULATION_SHAPING_FSK_BT_03,
	SX1232_MODULATION_SHAPING_OOK_BITRATE,
	SX1232_MODULATION_SHAPING_OOK_TWO_BITRATE,
} SX1232_ModulationShaping;

// RF output pin.
typedef enum {
	SX1232_RF_OUTPUT_PIN_RFO,
	SX1232_RF_OUTPUT_PIN_PABOOST,
} SX1232_RfOutputPin;

// Data mode.
typedef enum {
	SX1232_DATA_MODE_PACKET,
	SX1232_DATA_MODE_CONTINUOUS
} SX1232_DataMode;

// RX bandwidth mantissa.
typedef enum {
	SX1232_RXBW_MANTISSA_16,
	SX1232_RXBW_MANTISSA_20,
	SX1232_RXBW_MANTISSA_24
} SX1232_RxBwMantissa;

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
} SX1232_RssiSampling;

/*** SX1232 functions ***/

void SX1232_Init(void);
#ifdef HW2_0
void SX1232_Tcxo(unsigned char tcxo_enable);
#endif

// Common settings.
void SX1232_SetOscillator(SX1232_Oscillator oscillator);
void SX1232_SetMode(SX1232_Mode mode);
void SX1232_SetModulation(SX1232_Modulation modulation, SX1232_ModulationShaping modulation_shaping);
void SX1232_SetRfFrequency(unsigned int rf_frequency_hz);
void SX1232_EnableFastFrequencyHopping(void);
unsigned int SX1232_GetRfFrequency(void);
void SX1232_SetFskDeviation(unsigned short fsk_deviation_hz);
void SX1232_SetBitRate(unsigned int bit_rate_bps);
void SX1232_SetDataMode(SX1232_DataMode data_mode);
void SX1232_SetDioMapping(unsigned char diox, unsigned char diox_mapping);
unsigned short SX1232_GetIrqFlags(void);

// TX functions.
void SX1232_SelectRfOutputPin(SX1232_RfOutputPin rf_output_pin);
void SX1232_SetRfOutputPower(unsigned char rf_output_power_dbm);
void SX1232_EnableLowPnPll(void);
void SX1232_StartCw(void);
void SX1232_StopCw(void);

// RX functions.
void SX1232_SetRxBandwidth(SX1232_RxBwMantissa rxbw_mantissa, unsigned char rxbw_exponent);
void SX1232_EnableLnaBoost(unsigned char lna_boost_enable);
void SX1232_SetPreambleDetector(unsigned char preamble_length_bytes, unsigned char preamble_polarity);
void SX1232_SetSyncWord(unsigned char* sync_word, unsigned char sync_word_length_bytes);
void SX1232_SetDataLength(unsigned char data_length_bytes);
void SX1232_ConfigureRssi(signed char rssi_offset, SX1232_RssiSampling rssi_sampling);
unsigned char SX1232_GetRssi(void);
void SX1232_ReadFifo(unsigned char* rx_data, unsigned char rx_data_length);

#endif /* SX1232_H */
