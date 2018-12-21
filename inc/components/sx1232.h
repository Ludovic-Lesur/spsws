/*
 * sx1232.h
 *
 *  Created on: 20 june 2018
 *      Author: Ludovic
 */

#ifndef SX1232_H
#define SX1232_H

/*** SX1232 macros ***/

// Frequency range.
#define SX1232_RF_FREQUENCY_HZ_MIN				862000000
#define SX1232_RF_FREQUENCY_HZ_MAX				1020000000

// Output power ranges.
#define SX1232_OUTPUT_POWER_RFO_MIN				0
#define SX1232_OUTPUT_POWER_RFO_MAX				14
#define SX1232_OUTPUT_POWER_PABOOST_MIN			2
#define SX1232_OUTPUT_POWER_PABOOST_MAX			17

// Transmitter frequency hopping time.
#define SX1232_SPI_ACCESS_DURATION_US			8 // 8 bits @ SCK=1MHz.

/*** SX1232 structures ***/

// Oscillator configuration.
typedef enum {
	SX1232_QUARTZ,
	SX1232_TCXO
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

// Bit rate.
typedef enum {
	// Note: 100bps can't be programmed.
	SX1232_BITRATE_600BPS
} SX1232_BitRate;

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

/*** SX1232 functions ***/

void SX1232_Init(void);
// Common settings.
void SX1232_SetOscillator(SX1232_Oscillator oscillator);
void SX1232_SetMode(SX1232_Mode mode);
void SX1232_SetModulation(SX1232_Modulation modulation);
void SX1232_SetRfFrequency(unsigned int rf_frequency_hz);
unsigned int SX1232_GetRfFrequency(void);
void SX1232_SetBitRate(SX1232_BitRate bit_rate);
void SX1232_SetDataMode(SX1232_DataMode data_mode);
// TX functions.
void SX1232_SelectRfOutputPin(SX1232_RfOutputPin rf_output_pin);
void SX1232_SetRfOutputPower(unsigned char rf_output_power_dbm);
void SX1232_StartCw(unsigned int frequency_hz, unsigned char output_power_dbm);
void SX1232_StopCw(void);

#endif /* SX1232_H */
