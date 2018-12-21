/*
 * sx1232.c
 *
 *  Created on: 20 june 2018
 *      Author: Ludovic
 */

#include "sx1232.h"

#include "gpio.h"
#include "mapping.h"
#include "spi.h"
#include "sx1232_reg.h"
#include "tim.h"

/*** SX1232 local macros ***/

// SX1232 oscillator frequency.
#define SX1232_FXOSC_HZ		32000000

/*** SX1232 local structures ***/

typedef struct {
	SX1232_RfOutputPin sx1232_rf_output_pin;
} SX1232_Context;

/*** SX1232 local global variables ***/

SX1232_Context sx1232_ctx;

/*** SX1232 local functions ***/

/* SX1232 SINGLE ACCESS WRITE FUNCTION.
 * @param addr:		Register address (7 bits).
 * @param valie:	Value to write in register.
 * @return:			None.
 */
void SX1232_WriteRegister(unsigned char addr, unsigned char value) {
	/* Check addr is a 7-bits value */
	if (addr < (0b1 << 7)) {
		/* Build SPI frame */
		unsigned char sx1232_spi_command = 0;
		sx1232_spi_command |= (0b1 << 7) | addr; // '1 A6 A5 A4 A3 A2 A1 A0' for a write access.
		/* Write access sequence */
		GPIO_Write(GPIO_SX1232_CS, 0); // Falling edge on CS pin.
		SPI_WriteByte(sx1232_spi_command);
		SPI_WriteByte(value);
		GPIO_Write(GPIO_SX1232_CS, 1); // Set CS pin.
	}
}

/* SX1232 SINGLE ACCESS READ FUNCTION.
 * @param addr:		Register address (7 bits).
 * @param value:	Pointer to byte that will contain the register Value to read.
 * @return:			None.
 */
void SX1232_ReadRegister(unsigned char addr, unsigned char* value) {
	/* Check addr is a 7-bits value */
	if (addr < (0b1 << 7)) {
		/* Build SPI frame */
		unsigned char sx1232_spi_command = 0;
		sx1232_spi_command |= addr; // '0 A6 A5 A4 A3 A2 A1 A0' for a read access.
		/* Write access sequence */
		GPIO_Write(GPIO_SX1232_CS, 0); // Falling edge on CS pin.
		SPI_WriteByte(sx1232_spi_command);
		SPI_ReadByte(0xFF, value);
		GPIO_Write(GPIO_SX1232_CS, 1); // Set CS pin.
	}
}

/*** SX1232 functions ***/

/* INIT SX1232 TRANSCEIVER.
 * @param:	None.
 * @return:	None.
 */
void SX1232_Init(void) {

	/* Init context */
	sx1232_ctx.sx1232_rf_output_pin = SX1232_RF_OUTPUT_PIN_RFO;

	/* Init SX1232 DIOx */
#ifdef USE_SX1232_DIOX
	GPIO_Configure(GPIO_SX1232_DIOX, Output, PushPull, LowSpeed, NoPullUpNoPullDown);
#endif
	GPIO_Configure(GPIO_SX1232_DIO0, Input, PushPull, LowSpeed, NoPullUpNoPullDown);
}

/* SELECT SX1232 OSCILLATOR CONFIGURATION.
 * @param oscillator:	Type of external oscillator used (see SX1232_Oscillator enumeration in sx1232.h).
 * @return:				None.
 */
void SX1232_SetOscillator(SX1232_Oscillator oscillator) {

	/* Configure SPI */
	SPI_SetClockPolarity(0);

	/* Select oscillator */
	switch (oscillator) {
	case SX1232_QUARTZ:
		// Enable quartz input.
		SX1232_WriteRegister(SX1232_REG_TCXO, 0x09);
		break;
	case SX1232_TCXO:
		// Enable TCXO input.
		SX1232_WriteRegister(SX1232_REG_TCXO, 0x19);
		break;
	default:
		break;
	}

	/* Trigger RC oscillator calibration */
	SX1232_WriteRegister(SX1232_REG_OSC, 0x0F);
}

/* SET SX1232 TRANSCEIVER MODE.
 * @param mode: Mode to enter (see SX1232_Mode enumeration in sx1232.h).
 * @return:		None.
 */
void SX1232_SetMode(SX1232_Mode mode) {

	/* Configure SPI */
	SPI_SetClockPolarity(0);

	/* Read OP mode register */
	unsigned char op_mode_reg_value = 0;
	SX1232_ReadRegister(SX1232_REG_OPMODE, &op_mode_reg_value);
	op_mode_reg_value &= 0xF8; // Reset bits 0-2.

	/* Program transceiver mode */
	switch (mode) {
	case SX1232_MODE_SLEEP:
		// Allready done by previous reset.
		break;
	case SX1232_MODE_STANDBY:
		// Mode = '001'.
		op_mode_reg_value |= 0x01;
		break;
	case SX1232_MODE_FSTX:
		// Mode = '010'.
		op_mode_reg_value |= 0x02;
		break;
	case SX1232_MODE_TX:
		// Mode = '011'.
		op_mode_reg_value |= 0x03;
		break;
	case SX1232_MODE_FSRX:
		// Mode = '100'.
		op_mode_reg_value |= 0x04;
		break;
	case SX1232_MODE_RX:
		// Mode = '101'.
		op_mode_reg_value |= 0x05;
		break;
	default:
		break;
	}
	SX1232_WriteRegister(SX1232_REG_OPMODE, op_mode_reg_value);
}

/* SET SX1232 MODULATION.
 * @param modulation: 	Modulation scheme (see SX1232_Modulation enumeration in sx1232.h).
 * @return:				None.
 */
void SX1232_SetModulation(SX1232_Modulation modulation) {

	/* Configure SPI */
	SPI_SetClockPolarity(0);

	/* Read OP mode register */
	unsigned char op_mode_reg_value = 0;
	SX1232_ReadRegister(SX1232_REG_OPMODE, &op_mode_reg_value);
	op_mode_reg_value &= 0x9F; // Reset bits 5-6.

	/* Program modulation */
	switch (modulation) {
	case SX1232_MODULATION_FSK:
		// Modulation type = '00'.
		// Allready done by previous reset.
		break;
	case SX1232_MODULATION_OOK:
		// Modulation type = '01'.
		op_mode_reg_value |= 0x20;
		break;
	default:
		break;
	}
	SX1232_WriteRegister(SX1232_REG_OPMODE, op_mode_reg_value);
}

/* SET SX1232 RF FREQUENCY.
 * @param frequency_hz:	Transceiver frequency in Hz.
 * @return:				None.
 */
void SX1232_SetRfFrequency(unsigned int rf_frequency_hz) {

	/* Configure SPI */
	SPI_SetClockPolarity(0);

	/* Program RF frequency */
	unsigned long long frf_reg_value = (0b1 << 19);
	frf_reg_value *= rf_frequency_hz;
	frf_reg_value /= SX1232_FXOSC_HZ;
	SX1232_WriteRegister(SX1232_REG_FRFMSB, ((frf_reg_value & 0x00FF0000) >> 16));
	SX1232_WriteRegister(SX1232_REG_FRFMID, ((frf_reg_value & 0x0000FF00) >> 8));
	SX1232_WriteRegister(SX1232_REG_FRFLSB, (frf_reg_value & 0x000000FF));
}

/* GET EFFECTIVE RF FREQUENCY.
 * @param:					None.
 * @return rf_frequency_hz:	Effective programmed RF frequency in Hz.
 */
unsigned int SX1232_GetRfFrequency(void) {
	unsigned char byte_value = 0;
	unsigned int frf_reg_value = 0;
	SX1232_ReadRegister(SX1232_REG_FRFMSB, &byte_value);
	frf_reg_value |= (byte_value << 16);
	SX1232_ReadRegister(SX1232_REG_FRFMID, &byte_value);
	frf_reg_value |= (byte_value << 8);
	SX1232_ReadRegister(SX1232_REG_FRFLSB, &byte_value);
	frf_reg_value |= (byte_value << 0);

	unsigned long long rf_frequency_hz = ((unsigned long long) SX1232_FXOSC_HZ) * ((unsigned long long) frf_reg_value);
	rf_frequency_hz /= (0b1 << 19);

	return ((unsigned int) rf_frequency_hz);
}

/* SET SX1232 BIT RATE.
 * @param bit_rate: Bit rate to program (see SX1232_BitRate enumeration in sx1232.h).
 * @return:			None.
 */
void SX1232_SetBitRate(SX1232_BitRate bit_rate) {

	/* Configure SPI */
	SPI_SetClockPolarity(0);

	/* Set BitRate and BitRateFrac registers */
	switch (bit_rate) {
	case SX1232_BITRATE_600BPS:
		// 600 bits/s.
		// With FXOSX=32MHz, BR=53333, BRF=5: bit rate = FXOSC / (BR + BRF/16) = 600.0002 bps.
		SX1232_WriteRegister(SX1232_REG_BITRATEMSB, 0xD0);
		SX1232_WriteRegister(SX1232_REG_BITRATELSB, 0x55);
		SX1232_WriteRegister(SX1232_REG_BITRATEFRAC, 0x05);
		break;
	default:
		break;
	}
}

/* SET DATA MODE.
 * @param rf_output_power_dbm:	RF output power in dBm.
 * @return:						None.
 */
void SX1232_SetDataMode(SX1232_DataMode data_mode) {

	/* Configure SPI */
	SPI_SetClockPolarity(0);

	/* Read Packet config 2 register */
	unsigned char packet_config2_reg_value = 0;
	SX1232_ReadRegister(SX1232_REG_PACKETCONFIG2, &packet_config2_reg_value);
	packet_config2_reg_value &= 0xBF;

	/* Program data mode */
	switch (data_mode) {
	case SX1232_DATA_MODE_PACKET:
		// Data mode = '1'.
		packet_config2_reg_value |= 0x40;
		break;
	case SX1232_DATA_MODE_CONTINUOUS:
		// Data mode = '0'.
		// Allready done by previous reset.
		break;
	default:
		break;
	}
	SX1232_WriteRegister(SX1232_REG_PACKETCONFIG2, packet_config2_reg_value);
}

/* SELECT SX1232 RF OUTPUT PIN.
 * @param rf_outpu_pin:	RF output pin to select (see SX1232_RfOutputPin enumeration in sx1232.h).
 * @return:				None.
 */
void SX1232_SelectRfOutputPin(SX1232_RfOutputPin rf_output_pin) {

	/* Configure SPI */
	SPI_SetClockPolarity(0);

	/* Read PA config register */
	unsigned char pa_config_reg_value = 0;
	SX1232_ReadRegister(SX1232_REG_PACONFIG, &pa_config_reg_value);

	/* Program RF output pin */
	switch (rf_output_pin) {
	case SX1232_RF_OUTPUT_PIN_RFO:
		// PaSelect = '0'.
		pa_config_reg_value &= 0x7F;
		sx1232_ctx.sx1232_rf_output_pin = SX1232_RF_OUTPUT_PIN_RFO;
		break;
	case SX1232_RF_OUTPUT_PIN_PABOOST:
		// PaSelect = '1'.
		pa_config_reg_value |= 0x80;
		sx1232_ctx.sx1232_rf_output_pin = SX1232_RF_OUTPUT_PIN_PABOOST;
		break;
	default:
		break;
	}
	SX1232_WriteRegister(SX1232_REG_PACONFIG, pa_config_reg_value);
}

/* SET RF OUTPUT POWER.
 * @param rf_output_power_dbm:	RF output power in dBm.
 * @return:						None.
 */
void SX1232_SetRfOutputPower(unsigned char rf_output_power_dbm) {

	/* Configure SPI */
	SPI_SetClockPolarity(0);

	/* Read PA config register */
	unsigned char pa_config_reg_value = 0;
	SX1232_ReadRegister(SX1232_REG_PACONFIG, &pa_config_reg_value);
	pa_config_reg_value &= 0xF0; // Reset bits 0-3.

	/* Program RF output power */
	unsigned char effective_output_power_dbm = rf_output_power_dbm;
	switch (sx1232_ctx.sx1232_rf_output_pin) {
	case SX1232_RF_OUTPUT_PIN_RFO:
		// Ensure parameter is reachable.
		if (effective_output_power_dbm < SX1232_OUTPUT_POWER_RFO_MIN) {
			effective_output_power_dbm = SX1232_OUTPUT_POWER_RFO_MIN;
		}
		if (effective_output_power_dbm > SX1232_OUTPUT_POWER_RFO_MAX) {
			effective_output_power_dbm = SX1232_OUTPUT_POWER_RFO_MAX;
		}
		// Pout = -1 + OutputPower [dBm].
		pa_config_reg_value |= (effective_output_power_dbm + 1) & 0x0F;
		break;
	case SX1232_RF_OUTPUT_PIN_PABOOST:
		// Ensure parameter is reachable.
		if (effective_output_power_dbm < SX1232_OUTPUT_POWER_PABOOST_MIN) {
			effective_output_power_dbm = SX1232_OUTPUT_POWER_PABOOST_MIN;
		}
		if (effective_output_power_dbm > SX1232_OUTPUT_POWER_PABOOST_MAX) {
			effective_output_power_dbm = SX1232_OUTPUT_POWER_PABOOST_MAX;
		}
		// Pout = 2 + OutputPower [dBm].
		pa_config_reg_value |= (effective_output_power_dbm - 2) & 0x0F;
		break;
	default:
		break;
	}
	SX1232_WriteRegister(SX1232_REG_PACONFIG, pa_config_reg_value);
}

/* START CONTINUOUS WAVE OUTPUT.
 * @param frequency_hz:		CW frequency in Hz.
 * @param output_power_dBm:	CW output power in dBm.
 * @return:					None.
 */
void SX1232_StartCw(unsigned int frequency_hz, unsigned char output_power_dbm) {
	// Enable TCXO.
	SX1232_SetOscillator(SX1232_TCXO);
	TIM22_WaitMilliseconds(5); // Wait TS_OSC=250µs typical.
	// Enable low PN PLL.
	unsigned char reg_value = 0;
	SX1232_ReadRegister(SX1232_REG_PARAMP, &reg_value);
	SX1232_WriteRegister(SX1232_REG_PARAMP, (reg_value & 0xEF));
	// Enable fast frequency hopping.
	SX1232_ReadRegister(SX1232_REG_PLLHOP, &reg_value);
	SX1232_WriteRegister(SX1232_REG_PLLHOP, (reg_value | 0x80));
	// Configure radio parameters.
	SX1232_SetModulation(SX1232_MODULATION_OOK);
	SX1232_SetRfFrequency(frequency_hz);
	SX1232_SetRfOutputPower(output_power_dbm);
	// Use TX continuous mode.
	SX1232_SetDataMode(SX1232_DATA_MODE_CONTINUOUS);
	GPIO_Write(GPIO_SX1232_DIOX, 1);
	// Start radio.
	SX1232_SetMode(SX1232_MODE_FSTX);
	TIM22_WaitMilliseconds(5); // Wait TS_FS=60µs typical.
	SX1232_SetMode(SX1232_MODE_TX);
	TIM22_WaitMilliseconds(5); // Wait TS_TR=120µs typical.
}

/* STOP CONTINUOUS WAVE OUTPUT.
 * @param:	None.
 * @return:	None.
 */
void SX1232_StopCw(void) {
	// Stop data signal and radio.
	GPIO_Write(GPIO_SX1232_DIOX, 0);
	SX1232_SetMode(SX1232_MODE_STANDBY);
}
