/*
 * sx1232.c
 *
 *  Created on: 20 june 2018
 *      Author: Ludo
 */

#include "sx1232.h"

#include "gpio.h"
#include "lptim.h"
#include "mapping.h"
#include "spi.h"
#include "sx1232_reg.h"
#include "tim.h"

/*** SX1232 local macros ***/

// SX1232 oscillator frequency.
#define SX1232_FXOSC_HZ							32000000
#define SX1232_SYNC_WORD_MAXIMUM_LENGTH_BYTES	8

// SX1232 minimum and maximum bit rate.
#define SX1232_BIT_RATE_BPS_MIN					(SX1232_FXOSC_HZ / ((1 << 16) - 1))
#define SX1232_BIT_RATE_BPS_MAX					300000

// SX1232 maximum preamble length.
#define SX1232_PREAMBLE_LENGTH_BYTES_MAX		3

/*** SX1232 local structures ***/

typedef struct {
	SX1232_RfOutputPin sx1232_rf_output_pin;
	signed char sx1232_rssi_offset;
} SX1232_Context;

/*** SX1232 local global variables ***/

static SX1232_Context sx1232_ctx;

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
		GPIO_Write(&GPIO_SX1232_CS, 0); // Falling edge on CS pin.
		SPI1_WriteByte(sx1232_spi_command);
		SPI1_WriteByte(value);
		GPIO_Write(&GPIO_SX1232_CS, 1); // Set CS pin.
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
		GPIO_Write(&GPIO_SX1232_CS, 0); // Falling edge on CS pin.
		SPI1_WriteByte(sx1232_spi_command);
		SPI1_ReadByte(0xFF, value);
		GPIO_Write(&GPIO_SX1232_CS, 1); // Set CS pin.
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
	sx1232_ctx.sx1232_rssi_offset = 0;

	/* Init SX1232 DIOx */
#ifdef HW1_0
#ifdef USE_SX1232_DIOX
	GPIO_Configure(&GPIO_SX1232_DIOX, Output, PushPull, LowSpeed, NoPullUpNoPullDown);
#endif
#endif
#ifdef HW2_0
	GPIO_Configure(&GPIO_SX1232_DIO2, Output, PushPull, LowSpeed, NoPullUpNoPullDown);
#endif
	GPIO_Configure(&GPIO_SX1232_DIO0, Input, PushPull, LowSpeed, NoPullUpNoPullDown);

#ifdef HW2_0
	/* Init 32MHz TCXO power control */
	GPIO_Configure(&GPIO_TCXO32_POWER_ENABLE, Output, PushPull, LowSpeed, NoPullUpNoPullDown);
#endif
}

#ifdef HW2_0
/* SWITCH SX1232 EXTERNAL TCXO ON OR OFF.
 * @param tcxo_enable:	Power down 32MHz TCXO if 0, power on otherwise.
 * @return:				None.
 */
void SX1232_Tcxo(unsigned char tcxo_enable) {

	/* Update power control */
	if (tcxo_enable == 0) {
		GPIO_Write(&GPIO_TCXO32_POWER_ENABLE, 0);
	}
	else {
		GPIO_Write(&GPIO_TCXO32_POWER_ENABLE, 1);
		// Wait for TCXO to warm-up.
		LPTIM1_DelayMilliseconds(100);
	}
}
#endif

/* SELECT SX1232 OSCILLATOR CONFIGURATION.
 * @param oscillator:	Type of external oscillator used (see SX1232_Oscillator enumeration in sx1232.h).
 * @return:				None.
 */
void SX1232_SetOscillator(SX1232_Oscillator oscillator) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

	/* Select oscillator */
	switch (oscillator) {
	case SX1232_OSCILLATOR_QUARTZ:
		// Enable quartz input.
		SX1232_WriteRegister(SX1232_REG_TCXO, 0x09);
		break;
	case SX1232_OSCILLATOR_TCXO:
		// Enable TCXO input.
		SX1232_WriteRegister(SX1232_REG_TCXO, 0x19);
		break;
	default:
		break;
	}

	/* Wait TS_OSC = 250µs typical */
	LPTIM1_DelayMilliseconds(5);

	/* Trigger RC oscillator calibration */
	SX1232_WriteRegister(SX1232_REG_OSC, 0x0F);
}

/* SET SX1232 TRANSCEIVER MODE.
 * @param mode: Mode to enter (see SX1232_Mode enumeration in sx1232.h).
 * @return:		None.
 */
void SX1232_SetMode(SX1232_Mode mode) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

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

/* CONFIGURE SX1232 MODULATION.
 * @param modulation: 			Modulation scheme (see SX1232_Modulation enumeration in sx1232.h).
 * @param modulation_shaping: 	Modulation shaping (see SX1232_ModulationShaping enumeration in sx1232.h).
 * @return:						None.
 */
void SX1232_SetModulation(SX1232_Modulation modulation, SX1232_ModulationShaping modulation_shaping) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

	/* Read OP mode register */
	unsigned char op_mode_reg_value = 0;
	SX1232_ReadRegister(SX1232_REG_OPMODE, &op_mode_reg_value);
	op_mode_reg_value &= 0x87; // Reset bits 3-6.

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

	/* Program modulation shaping */
	switch (modulation_shaping) {
	case SX1232_MODULATION_SHAPING_NONE:
		// Modulation shaping = '00'.
		// Allready done by previous reset.
		break;
	case SX1232_MODULATION_SHAPING_FSK_BT_1:
	case SX1232_MODULATION_SHAPING_OOK_BITRATE:
		// Modulation shaping = '01'.
		op_mode_reg_value |= 0x08;
		break;
	case SX1232_MODULATION_SHAPING_FSK_BT_05:
	case SX1232_MODULATION_SHAPING_OOK_TWO_BITRATE:
		// Modulation shaping = '10'.
		op_mode_reg_value |= 0x10;
		break;
	case SX1232_MODULATION_SHAPING_FSK_BT_03:
		// Modulation shaping = '11'.
		op_mode_reg_value |= 0x18;
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

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

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

/* SET FSK DEVIATION.
 * @param fsk_deviation_hz:	FSK deviation in Hz.
 * @return:					None.
 */
void SX1232_SetFskDeviation(unsigned short fsk_deviation_hz) {

	// Check value is on 14-bits.
	if (fsk_deviation_hz < (0b1 << 14)) {

#ifdef HW1_0
		/* Configure SPI */
		SPI1_SetClockPolarity(0);
#endif

		/* Program FSK deviation */
		unsigned long long fdev_reg_value = (0b1 << 19);
		fdev_reg_value *= fsk_deviation_hz;
		fdev_reg_value /= SX1232_FXOSC_HZ;
		SX1232_WriteRegister(SX1232_REG_FDEVMSB, ((fdev_reg_value & 0x00003F00) >> 8));
		SX1232_WriteRegister(SX1232_REG_FDEVLSB, ((fdev_reg_value & 0x000000FF) >> 0));
	}
}

/* SET SX1232 BIT RATE.
 * @param bit_rate: Bit rate to program in bit per seconds (bps).
 * @return:			None.
 */
void SX1232_SetBitRate(unsigned int bit_rate_bps) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

	/* Check parameter */
	unsigned int local_bit_rate_bps = bit_rate_bps;
	if (local_bit_rate_bps < SX1232_BIT_RATE_BPS_MIN) {
		local_bit_rate_bps = SX1232_BIT_RATE_BPS_MIN;
	}
	if (local_bit_rate_bps > SX1232_BIT_RATE_BPS_MAX) {
		local_bit_rate_bps = SX1232_BIT_RATE_BPS_MAX;
	}

	/* Set BitRate register */
	SX1232_WriteRegister(SX1232_REG_BITRATEFRAC, 0x00);
	// Compute register value: BR = FXOSC / bit_rate.
	unsigned int bit_rate_reg_value = SX1232_FXOSC_HZ / local_bit_rate_bps;
	SX1232_WriteRegister(SX1232_REG_BITRATEMSB, ((bit_rate_reg_value & 0x0000FF00) >> 8));
	SX1232_WriteRegister(SX1232_REG_BITRATELSB, ((bit_rate_reg_value & 0x000000FF) >> 0));
}

/* SET DATA MODE.
 * @param rf_output_power_dbm:	RF output power in dBm.
 * @return:						None.
 */
void SX1232_SetDataMode(SX1232_DataMode data_mode) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

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

/* CONFIGURE SX1232 DIO MAPPING.
 * @param dio:			GPIO to configure (0 to 5).
 * @param dio_mapping:	GPIO function (2-bits value).
 */
void SX1232_SetDioMapping(unsigned char dio, unsigned char dio_mapping) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

	/* Check parameters */
	if ((dio < 6) && (dio_mapping < 4)) {

		/* Configure proper register */
		unsigned char dio_mapping_reg_value = 0;
		switch (dio) {
		// DIO0.
		case 0:
			SX1232_ReadRegister(SX1232_REG_DIOMAPPING1, &dio_mapping_reg_value);
			dio_mapping_reg_value &= 0x3F;
			dio_mapping_reg_value |= (dio_mapping << 6);
			SX1232_WriteRegister(SX1232_REG_DIOMAPPING1, dio_mapping_reg_value);
			break;
		// DIO1.
		case 1:
			SX1232_ReadRegister(SX1232_REG_DIOMAPPING1, &dio_mapping_reg_value);
			dio_mapping_reg_value &= 0xCF;
			dio_mapping_reg_value |= (dio_mapping << 4);
			SX1232_WriteRegister(SX1232_REG_DIOMAPPING1, dio_mapping_reg_value);
			break;
		// DIO2.
		case 2:
			SX1232_ReadRegister(SX1232_REG_DIOMAPPING1, &dio_mapping_reg_value);
			dio_mapping_reg_value &= 0xF3;
			dio_mapping_reg_value |= (dio_mapping << 2);
			SX1232_WriteRegister(SX1232_REG_DIOMAPPING1, dio_mapping_reg_value);
			break;
		// DIO3.
		case 3:
			SX1232_ReadRegister(SX1232_REG_DIOMAPPING1, &dio_mapping_reg_value);
			dio_mapping_reg_value &= 0xFC;
			dio_mapping_reg_value |= (dio_mapping << 0);
			SX1232_WriteRegister(SX1232_REG_DIOMAPPING1, dio_mapping_reg_value);
			break;
		// DIO0.
		case 4:
			SX1232_ReadRegister(SX1232_REG_DIOMAPPING2, &dio_mapping_reg_value);
			dio_mapping_reg_value &= 0x3F;
			dio_mapping_reg_value |= (dio_mapping << 6);
			SX1232_WriteRegister(SX1232_REG_DIOMAPPING2, dio_mapping_reg_value);
			break;
		// DIO0.
		case 5:
			SX1232_ReadRegister(SX1232_REG_DIOMAPPING2, &dio_mapping_reg_value);
			dio_mapping_reg_value &= 0xCF;
			dio_mapping_reg_value |= (dio_mapping << 4);
			SX1232_WriteRegister(SX1232_REG_DIOMAPPING2, dio_mapping_reg_value);
			break;
		}
	}
}

/* READ SX1232 IRQ FLAGS REGISTERS.
 * @param:	None.
 * @return 	irq_flags_value:	16-bits value read as [REG_IRQFLASG1 REG_IRQFLASG2].
 */
unsigned short SX1232_GetIrqFlags(void) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

	/* Read registers */
	unsigned char reg_value = 0;
	unsigned short irq_flags_value = 0;
	SX1232_ReadRegister(SX1232_REG_IRQFLAGS1, &reg_value);
	irq_flags_value |= (reg_value << 8);
	SX1232_ReadRegister(SX1232_REG_IRQFLAGS2, &reg_value);
	irq_flags_value |= (reg_value << 0);

	return irq_flags_value;
}

/* SELECT SX1232 RF OUTPUT PIN.
 * @param rf_outpu_pin:	RF output pin to select (see SX1232_RfOutputPin enumeration in sx1232.h).
 * @return:				None.
 */
void SX1232_SelectRfOutputPin(SX1232_RfOutputPin rf_output_pin) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

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

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

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

/* ENABLE LOW PHASE NOISE PLL.
 * @param:	None.
 * @return:	None.
 */
void SX1232_EnableLowPnPll(void) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

	/* Program register */
	unsigned char reg_value = 0;
	SX1232_ReadRegister(SX1232_REG_PARAMP, &reg_value);
	SX1232_WriteRegister(SX1232_REG_PARAMP, (reg_value & 0xEF));
}

/* ENABLE FAST FREQUENCY HOPPING.
 * @param:	None.
 * @return:	None.
 */
void SX1232_EnableFastFrequencyHopping(void) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

	/* Program register */
	unsigned char reg_value = 0;
	SX1232_ReadRegister(SX1232_REG_PLLHOP, &reg_value);
	SX1232_WriteRegister(SX1232_REG_PLLHOP, (reg_value | 0x80));
}

/* START CONTINUOUS WAVE OUTPUT.
 * @param:	None.
 * @return:	None.
 */
void SX1232_StartCw(void) {
	// Start data signal.
#ifdef HW1_0
	GPIO_Write(&GPIO_SX1232_DIOX, 1);
#endif
#ifdef HW2_0
	GPIO_Write(&GPIO_SX1232_DIO2, 1);
#endif
	// Start radio.
	SX1232_SetMode(SX1232_MODE_FSTX);
	LPTIM1_DelayMilliseconds(5); // Wait TS_FS=60µs typical.
	SX1232_SetMode(SX1232_MODE_TX);
	LPTIM1_DelayMilliseconds(5); // Wait TS_TR=120µs typical.
}

/* STOP CONTINUOUS WAVE OUTPUT.
 * @param:	None.
 * @return:	None.
 */
void SX1232_StopCw(void) {
	// Stop data signal and radio.
#ifdef HW1_0
	GPIO_Write(&GPIO_SX1232_DIOX, 0);
#endif
#ifdef HW2_0
	GPIO_Write(&GPIO_SX1232_DIO2, 0);
#endif
	SX1232_SetMode(SX1232_MODE_STANDBY);
}

/* SET SX1232 RX BANDWIDTH.
 * @param rxbw_mantissa:	RXBW mantissa (see p.30 of SX1232 datasheet).
 * @param rxbw_exponenta:	RXBW exponent (see p.30 of SX1232 datasheet).
 * @return:					None.
 */
void SX1232_SetRxBandwidth(SX1232_RxBwMantissa rxbw_mantissa, unsigned char rxbw_exponent) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

	/* Read register */
	unsigned char rxbw_reg_value = 0;
	SX1232_ReadRegister(SX1232_REG_RXBW, &rxbw_reg_value);

	/* Program mantissa */
	rxbw_reg_value &= 0xE0; // Reset bits 0-4.
	rxbw_reg_value |= (rxbw_mantissa & 0x00000003) << 3;

	/* Program exponent */
	unsigned char local_rxbw_exponent = rxbw_exponent;
	if (local_rxbw_exponent > SX1232_RXBW_EXPONENT_MAX) {
		local_rxbw_exponent = SX1232_RXBW_EXPONENT_MAX;
	}
	rxbw_reg_value |= local_rxbw_exponent;

	/* Program register */
	SX1232_WriteRegister(SX1232_REG_RXBW, rxbw_reg_value);
}

/* CONTROL SX1232 LNA BOOST.
 * @param lna_boost_enable:	Enable (1) or disable (0) LNA boost.
 * @return:					None.
 */
void SX1232_EnableLnaBoost(unsigned char lna_boost_enable) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

	/* Program register */
	unsigned char lna_reg_value = 0;
	SX1232_ReadRegister(SX1232_REG_LNA, &lna_reg_value);
	if (lna_boost_enable != 0) {
		// LnaBoost = '11'.
		lna_reg_value |= 0x03;
	}
	else {
		// LnaBoost = '00'.
		lna_reg_value &= 0xFC;
	}
	SX1232_WriteRegister(SX1232_REG_LNA, lna_reg_value);
}

/* ENABLE PREAMBLE DETECTOR.
 * @param preamble_length_bytes:	Preamble length in bytes (0 disables preamble detector).
 * @param preamble_polarity:		Use 0xAA (0) or 0x55 (1) as preamble byte.
 * @return:							None.
 */
void SX1232_SetPreambleDetector(unsigned char preamble_length_bytes, unsigned char preamble_polarity) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

	/* Set length and enable */
	unsigned char reg_value = 0;
	SX1232_ReadRegister(SX1232_REG_PREAMBLEDETECT, &reg_value);
	if (preamble_length_bytes == 0) {
		reg_value &= 0x7F; // Disable preamble detector.
	}
	else {
		unsigned char local_preamble_length_bytes = preamble_length_bytes;
		if (local_preamble_length_bytes > SX1232_PREAMBLE_LENGTH_BYTES_MAX) {
			local_preamble_length_bytes = SX1232_PREAMBLE_LENGTH_BYTES_MAX;
		}
		reg_value &= 0x9F; // Reset bits 5-6.
		reg_value |= (preamble_length_bytes - 1);
		reg_value |= 0x80; // Enable preamble detector.
	}
	SX1232_WriteRegister(SX1232_REG_PREAMBLEDETECT, reg_value);

	/* Set polarity */
	SX1232_ReadRegister(SX1232_REG_SYNCCONFIG, &reg_value);
	if (preamble_polarity == 0) {
		SX1232_WriteRegister(SX1232_REG_SYNCCONFIG, (reg_value & 0xDF));
	}
	else {
		SX1232_WriteRegister(SX1232_REG_SYNCCONFIG, (reg_value | 0x20));
	}
}

/* CONFIGURE RX SYNCHRONIZATION WORD.
 * @param sync_word:				Synchronization word.
 * @param sync_word_length_bytes:	Synchronization word length in bytes.
 * @return:							None.
 */
void SX1232_SetSyncWord(unsigned char* sync_word, unsigned char sync_word_length_bytes) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

	/* Check parameters */
	if ((sync_word_length_bytes > 0) && (sync_word_length_bytes <= SX1232_SYNC_WORD_MAXIMUM_LENGTH_BYTES)) {

		/* Set syncronization word length */
		unsigned char reg_value = 0;
		SX1232_ReadRegister(SX1232_REG_SYNCCONFIG, &reg_value);
		reg_value &= 0xF8; // Reset bits 0-2.
		reg_value |= ((sync_word_length_bytes - 1) & 0x07);
		reg_value &= 0x3F; // Disable receiver auto_restart.
		reg_value |= 0x10; // Enable syncronization word detector.
		SX1232_WriteRegister(SX1232_REG_SYNCCONFIG, reg_value);

		/* Fill synchronization word */
		unsigned char byte_idx = 0;
		for (byte_idx=0 ; byte_idx<sync_word_length_bytes ; byte_idx++) {
			// Warning: this loop is working because registers addresses are adjacent.
			SX1232_WriteRegister((SX1232_REG_SYNCVALUE1 + byte_idx), sync_word[byte_idx]);
		}
	}
}

/* SET RX PAYLOAD LENGTH.
 * @param data_length_bytes:	Data length to receive in bytes.
 * @return:						None.
 */
void SX1232_SetDataLength(unsigned char data_length_bytes) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

	/* Use fixed length, no CRC computation, do not clear FIFO when CRC fails */
	SX1232_WriteRegister(SX1232_REG_PACKETCONFIG1, 0x08);

	/* Set data length */
	SX1232_WriteRegister(SX1232_REG_PAYLOADLENGTH, data_length_bytes);
}

/* CONFIGURE SX1232 RSSI MEASUREMENT.
 * @param rssi_offset: 		RSSI offset in dBm.
 * @param rssi_sampling:	Number of samples used to average RSSI value (see SX1232_RssiSampling enumeration in sx1232.h).
 * @return:					None.
 */
void SX1232_ConfigureRssi(signed char rssi_offset, SX1232_RssiSampling rssi_sampling) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

	/* Configure offset */
	sx1232_ctx.sx1232_rssi_offset = rssi_offset;

	/* Configure sampling */
	unsigned char rssi_config_reg_value = 0; // Do not use internal RSSI offset.
	rssi_config_reg_value |= (rssi_sampling & 0x00000007);
}

/* READ SX1232 RSSI.
 * @param:	None.
 * @return:	Absolute RSSI value in dBm.
 */
unsigned char SX1232_GetRssi(void) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

	/* Read RSSI register and add offset */
	signed char rssi = 0;
	unsigned char rssi_reg_value = 0;
	SX1232_ReadRegister(SX1232_REG_RSSIVALUE, &rssi_reg_value);
	rssi = (rssi_reg_value / 2) + sx1232_ctx.sx1232_rssi_offset;

	return rssi;
}

/* READ SX1232 FIFO.
 * @param rx_data:			Byte array that will contain FIFO data.
 * @param rx_data_length:	Number of bytes to read in FIFO.
 */
void SX1232_ReadFifo(unsigned char* rx_data, unsigned char rx_data_length) {

#ifdef HW1_0
	/* Configure SPI */
	SPI1_SetClockPolarity(0);
#endif

	/* Access FIFO byte per byte */
	unsigned char byte_idx = 0;
	unsigned char rx_data_byte = 0;
	for (byte_idx=0 ; byte_idx<rx_data_length ; byte_idx++) {
		SX1232_ReadRegister(SX1232_REG_FIFO, &rx_data_byte);
		rx_data[byte_idx] = rx_data_byte;
	}
}
