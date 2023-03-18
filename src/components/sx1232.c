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
#include "types.h"

/*** SX1232 local macros ***/

// RF frequency range.
#define SX1232_RF_FREQUENCY_HZ_MIN				862000000
#define SX1232_RF_FREQUENCY_HZ_MAX				1020000000
// TCXO and output clock frequency (DIO5).
#define SX1232_FXOSC_HZ							32000000
#define SX1232_CLKOUT_PRESCALER					2
#define SX1232_CLKOUT_FREQUENCY_KHZ				((SX1232_FXOSC_HZ) / (SX1232_CLKOUT_PRESCALER * 1000))
// FSK deviation range.
#define SX1232_FSK_DEVIATION_MAX				16383 // 2^(14) - 1.
// Output power ranges.
#define SX1232_OUTPUT_POWER_RFO_MIN				0
#define SX1232_OUTPUT_POWER_RFO_MAX				17
#define SX1232_OUTPUT_POWER_PABOOST_MIN			2
#define SX1232_OUTPUT_POWER_PABOOST_MAX			20
// RSSI offset due to internal and external LNA (calibration).
#define SX1232_RSSI_OFFSET_DB					24
// SX1232 oscillator frequency.
#define SX1232_SYNC_WORD_MAXIMUM_LENGTH_BYTES	8
// SX1232 minimum and maximum bit rate.
#define SX1232_BIT_RATE_BPS_MIN					(SX1232_FXOSC_HZ / ((1 << 16) - 1))
#define SX1232_BIT_RATE_BPS_MAX					300000
// SX1232 maximum preamble length.
#define SX1232_PREAMBLE_LENGTH_BYTES_MAX		3
// SX1232 DIOs.
#define SX1232_DIO_NUMBER						6
#define SX1232_DIO_MAPPING_MAX_VALUE			4
// SX1232 FIFO.
#define SX1232_FIFO_LENGTH						64

/*** SX1232 local global variables ***/

static SX1232_rf_output_pin_t sx1232_rf_output_pin;

/*** SX1232 local functions ***/

/* SX1232 SINGLE ACCESS WRITE FUNCTION.
 * @param addr:		Register address (7 bits).
 * @param valie:	Value to write in register.
 * @return status:	Function execution status.
 */
static SX1232_status_t _SX1232_write_register(uint8_t addr, uint8_t value) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	uint8_t sx1232_spi_command = 0;
	// Check parameters.
	if (addr > SX1232_REG_LAST) {
		status = SX1232_ERROR_REGISTER_ADDRESS;
		goto errors;
	}
	// Build SPI frame.
	sx1232_spi_command |= (addr | 0x80); // '1 A6 A5 A4 A3 A2 A1 A0' for a write access.
#ifdef HW1_0
	SPI1_set_clock_polarity(0);
#endif
	// Write access sequence.
	GPIO_write(&GPIO_SX1232_CS, 0); // Falling edge on CS pin.
	spi_status = SPI1_write_byte(sx1232_spi_command);
	SPI1_status_check(SX1232_ERROR_BASE_SPI);
	spi_status = SPI1_write_byte(value);
	SPI1_status_check(SX1232_ERROR_BASE_SPI);
errors:
	GPIO_write(&GPIO_SX1232_CS, 1); // Set CS pin.
	return status;
}

/* SX1232 SINGLE ACCESS READ FUNCTION.
 * @param addr:		Register address (7 bits).
 * @param value:	Pointer to 8-bits value that will contain the register Value to read.
 * @return status:	Function execution status.
 */
static SX1232_status_t _SX1232_read_register(uint8_t addr, uint8_t* value) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	uint8_t sx1232_spi_command = 0;
	// Check parameters.
	if (addr > SX1232_REG_LAST) {
		status = SX1232_ERROR_REGISTER_ADDRESS;
		goto errors;
	}
	if (value == NULL) {
		status = SX1232_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Build SPI frame.
	sx1232_spi_command = (addr & 0x7F); // '0 A6 A5 A4 A3 A2 A1 A0' for a read access.
#ifdef HW1_0
	SPI1_set_clock_polarity(0);
#endif
	// Write access sequence.
	GPIO_write(&GPIO_SX1232_CS, 0); // Falling edge on CS pin.
	spi_status = SPI1_write_byte(sx1232_spi_command);
	SPI1_status_check(SX1232_ERROR_BASE_SPI);
	spi_status = SPI1_read_byte(0xFF, value);
	SPI1_status_check(SX1232_ERROR_BASE_SPI);
errors:
	GPIO_write(&GPIO_SX1232_CS, 1); // Set CS pin.
	return status;
}

/*** SX1232 functions ***/

/* INIT SX1232 TRANSCEIVER.
 * @param:	None.
 * @return:	None.
 */
void SX1232_init(void) {
	// Init context.
	sx1232_rf_output_pin = SX1232_RF_OUTPUT_PIN_RFO;
	// Init SX1232 DIOx.
	GPIO_configure(&GPIO_SX1232_DIO2, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SX1232_DIO0, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_TCXO32_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/* DISABLE ALL SX1232 GPIOs.
 * @param:	None.
 * @return:	None.
 */
void SX1232_disable(void) {
	// Disable GPIOs.
	GPIO_configure(&GPIO_SX1232_DIO0, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SX1232_DIO2, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_TCXO32_POWER_ENABLE, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/* SWITCH SX1232 EXTERNAL TCXO ON OR OFF.
 * @param tcxo_enable:	Power down 32MHz TCXO if 0, power on otherwise.
 * @return status:		Function execution status.
 */
SX1232_status_t SX1232_tcxo(uint8_t tcxo_enable) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// Update power control.
	GPIO_write(&GPIO_TCXO32_POWER_ENABLE, tcxo_enable);
	// Wait for TCXO to warm-up.
	lptim1_status = LPTIM1_delay_milliseconds(100, LPTIM_DELAY_MODE_STOP);
	LPTIM1_status_check(SX1232_ERROR_BASE_LPTIM);
errors:
	return status;
}

/* SELECT SX1232 OSCILLATOR CONFIGURATION.
 * @param oscillator:	External oscillator type.
 * @return status:		Function execution status.
 */
SX1232_status_t SX1232_set_oscillator(SX1232_oscillator_t oscillator) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// Select oscillator.
	switch (oscillator) {
	case SX1232_OSCILLATOR_QUARTZ:
		// Enable quartz input.
		_SX1232_write_register(SX1232_REG_TCXO, 0x09);
		break;
	case SX1232_OSCILLATOR_TCXO:
		// Enable TCXO input.
		_SX1232_write_register(SX1232_REG_TCXO, 0x19);
		break;
	default:
		status = SX1232_ERROR_OSCILLATOR;
		goto errors;
	}
	// Wait TS_OSC = 250us typical.
	lptim1_status = LPTIM1_delay_milliseconds(5, LPTIM_DELAY_MODE_STOP);
	LPTIM1_status_check(SX1232_ERROR_BASE_LPTIM);
	// Trigger RC oscillator calibration.
	status = _SX1232_write_register(SX1232_REG_OSC, 0x0F);
errors:
	return status;
}

/* SET SX1232 TRANSCEIVER MODE.
 * @param mode: 	Mode to enter.
 * @return status:	Function execution status.
 */
SX1232_status_t SX1232_set_mode(SX1232_mode_t mode) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t op_mode_reg_value = 0;
	// Read OP mode register.
	status = _SX1232_read_register(SX1232_REG_OPMODE, &op_mode_reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Reset bits 0-2.
	op_mode_reg_value &= 0xF8;
	// Compute register.
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
		status = SX1232_ERROR_MODE;
		goto errors;
	}
	// Programe register.
	status = _SX1232_write_register(SX1232_REG_OPMODE, op_mode_reg_value);
errors:
	return status;
}

/* CONFIGURE SX1232 MODULATION.
 * @param modulation: 			Modulation scheme.
 * @param modulation_shaping: 	Modulation shaping.
 * @return status:				Function execution status.
 */
SX1232_status_t SX1232_set_modulation(SX1232_modulation_t modulation, SX1232_modulation_shaping_t modulation_shaping) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t op_mode_reg_value = 0;
	// Read OP mode register.
	status = _SX1232_read_register(SX1232_REG_OPMODE, &op_mode_reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Reset bits 3-6.
	op_mode_reg_value &= 0x87;
	// Compute modulation.
	switch (modulation) {
	case SX1232_MODULATION_FSK:
		// Allready done by previous reset.
		break;
	case SX1232_MODULATION_OOK:
		// Modulation type = '01'.
		op_mode_reg_value |= 0x20;
		break;
	default:
		status = SX1232_ERROR_MODULATION;
		goto errors;
	}
	// Compute modulation shaping.
	switch (modulation_shaping) {
	case SX1232_MODULATION_SHAPING_NONE:
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
		status = SX1232_ERROR_MODULATION_SHAPING;
		goto errors;
	}
	// Programe register.
	status = _SX1232_write_register(SX1232_REG_OPMODE, op_mode_reg_value);
errors:
	return status;
}

/* SET SX1232 RF FREQUENCY.
 * @param rffrequency_hz:	Transceiver frequency in Hz.
 * @return status:			Function execution status.
 */
SX1232_status_t SX1232_set_rf_frequency(uint32_t rf_frequency_hz) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint64_t frf_reg_value = 0;
	// Check frequency range.
	if (rf_frequency_hz > SX1232_RF_FREQUENCY_HZ_MAX) {
		status = SX1232_ERROR_RF_FREQUENCY_OVERFLOW;
		goto errors;
	}
	if (rf_frequency_hz < SX1232_RF_FREQUENCY_HZ_MIN) {
		status = SX1232_ERROR_RF_FREQUENCY_UNDERFLOW;
		goto errors;
	}
	// Compute register.
	frf_reg_value = (0b1 << 19);
	frf_reg_value *= rf_frequency_hz;
	frf_reg_value /= SX1232_FXOSC_HZ;
	// Program register.
	status = _SX1232_write_register(SX1232_REG_FRFMSB, ((frf_reg_value & 0x00FF0000) >> 16));
	if (status != SX1232_SUCCESS) goto errors;
	status = _SX1232_write_register(SX1232_REG_FRFMID, ((frf_reg_value & 0x0000FF00) >> 8));
	if (status != SX1232_SUCCESS) goto errors;
	status = _SX1232_write_register(SX1232_REG_FRFLSB, ((frf_reg_value & 0x000000FF) >> 0));
errors:
	return status;
}

/* GET EFFECTIVE RF FREQUENCY.
 * @param rf_frequency_hz:	Pointer to 32-bits value that will contain effective programmed RF frequency in Hz.
 * @return status:			Function execution status.
 */
SX1232_status_t SX1232_get_rf_frequency(uint32_t* rf_frequency_hz) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t byte_value = 0;
	uint32_t frf_reg_value = 0;
	uint64_t local_rf_frequency_hz = 0;
	// Check parameter.
	if (rf_frequency_hz == NULL) {
		status = SX1232_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Read registers.
	status = _SX1232_read_register(SX1232_REG_FRFMSB, &byte_value);
	if (status != SX1232_SUCCESS) goto errors;
	frf_reg_value |= (byte_value << 16);
	status = _SX1232_read_register(SX1232_REG_FRFMID, &byte_value);
	if (status != SX1232_SUCCESS) goto errors;
	frf_reg_value |= (byte_value << 8);
	status = _SX1232_read_register(SX1232_REG_FRFLSB, &byte_value);
	if (status != SX1232_SUCCESS) goto errors;
	frf_reg_value |= (byte_value << 0);
	// Convert to Hz.
	local_rf_frequency_hz = ((uint64_t) SX1232_FXOSC_HZ) * ((uint64_t) frf_reg_value);
	local_rf_frequency_hz /= (0b1 << 19);
	// Convert to integer.
	(*rf_frequency_hz) = (uint32_t) local_rf_frequency_hz;
errors:
	return status;
}

/* ENABLE FAST FREQUENCY HOPPING.
 * @param:			None.
 * @return status:	Function execution status.
 */
SX1232_status_t SX1232_enable_fast_frequency_hopping(void) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t reg_value = 0;
	// Read register.
	status = _SX1232_read_register(SX1232_REG_PLLHOP, &reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Program register.
	status = _SX1232_write_register(SX1232_REG_PLLHOP, (reg_value | 0x80));
errors:
	return status;
}

/* SET FSK DEVIATION.
 * @param fsk_deviation_hz:	FSK deviation in Hz.
 * @return status:			Function execution status.
 */
SX1232_status_t SX1232_set_fsk_deviation(uint32_t fsk_deviation_hz) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint64_t fdev_reg_value = 0;
	// Check parameter.
	if (fsk_deviation_hz > SX1232_FSK_DEVIATION_MAX) {
		status = SX1232_ERROR_FSK_DEVIATION;
		goto errors;
	}
	// Compute register.
	fdev_reg_value = (0b1 << 19);
	fdev_reg_value *= fsk_deviation_hz;
	fdev_reg_value /= SX1232_FXOSC_HZ;
	// Program register.
	status = _SX1232_write_register(SX1232_REG_FDEVMSB, ((fdev_reg_value & 0x00003F00) >> 8));
	if (status != SX1232_SUCCESS) goto errors;
	status = _SX1232_write_register(SX1232_REG_FDEVLSB, ((fdev_reg_value & 0x000000FF) >> 0));
errors:
	return status;
}

/* SET SX1232 BIT RATE.
 * @param bit_rate: Bit rate to program in bit per seconds (bps).
 * @return status:	Function execution status.
 */
SX1232_status_t SX1232_set_bitrate(uint32_t bit_rate_bps) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint32_t bit_rate_reg_value = 0;
	// Check parameter.
	if (bit_rate_bps > SX1232_BIT_RATE_BPS_MAX) {
		status = SX1232_ERROR_BIT_RATE_OVERFLOW;
		goto errors;
	}
	if (bit_rate_bps < SX1232_BIT_RATE_BPS_MIN) {
		status = SX1232_ERROR_BIT_RATE_UNDERFLOW;
		goto errors;
	}
	// Compute register.
	bit_rate_reg_value = SX1232_FXOSC_HZ / bit_rate_bps;
	// Program register.
	status = _SX1232_write_register(SX1232_REG_BITRATEFRAC, 0x00);
	if (status != SX1232_SUCCESS) goto errors;
	status = _SX1232_write_register(SX1232_REG_BITRATEMSB, ((bit_rate_reg_value & 0x0000FF00) >> 8));
	if (status != SX1232_SUCCESS) goto errors;
	status = _SX1232_write_register(SX1232_REG_BITRATELSB, ((bit_rate_reg_value & 0x000000FF) >> 0));
errors:
	return status;
}

/* SET DATA MODE.
 * @param data_mode:	Data mode.
 * @return status:		Function execution status.
 */
SX1232_status_t SX1232_set_data_mode(SX1232_data_mode_t data_mode) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t packet_config2_reg_value = 0;
	// Read register.
	status = _SX1232_read_register(SX1232_REG_PACKETCONFIG2, &packet_config2_reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Reset bit 6.
	packet_config2_reg_value &= 0xBF;
	// Program data mode.
	switch (data_mode) {
	case SX1232_DATA_MODE_PACKET:
		// Data mode = '1'.
		packet_config2_reg_value |= 0x40;
		break;
	case SX1232_DATA_MODE_CONTINUOUS:
		// Allready done by previous reset.
		break;
	default:
		status = SX1232_ERROR_DATA_MODE;
		goto errors;
	}
	// Program register.
	status = _SX1232_write_register(SX1232_REG_PACKETCONFIG2, packet_config2_reg_value);
errors:
	return status;
}

/* CONFIGURE SX1232 DIO MAPPING.
 * @param dio:			DIO to configure (0 to 5).
 * @param dio_mapping:	DIO function (2-bits value).
 * @return status:		Function execution status.
 */
SX1232_status_t SX1232_set_dio_mapping(SX1232_dio_t dio, SX1232_dio_mapping_t dio_mapping) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t dio_mapping_reg_addr = 0;
	uint8_t dio_mapping_reg_value = 0;
	// Check parameters.
	if (dio >= SX1232_DIO_LAST) {
		status = SX1232_ERROR_DIO;
		goto errors;
	}
	if (dio_mapping >= SX1232_DIO_MAPPING_LAST) {
		status = SX1232_ERROR_DIO_MAPPING;
		goto errors;
	}
	// Select register and mask.
	dio_mapping_reg_addr = SX1232_REG_DIOMAPPING1 + (dio / 4);
	// Read register.
	status = _SX1232_read_register(dio_mapping_reg_addr, &dio_mapping_reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Compute register.
	dio_mapping_reg_value &= ~(0b11 << ((dio % 4) * 2));
	dio_mapping_reg_value |= (dio_mapping << ((dio % 4) * 2));
	// Write register.
	status = _SX1232_write_register(dio_mapping_reg_addr, dio_mapping_reg_value);
errors:
	return status;
}

/* READ SX1232 IRQ FLAGS REGISTERS.
 * @param irq_flags:	Pointer to 16-bits value read as [REG_IRQFLASG1 REG_IRQFLASG2].
 * @return status:		Function execution status.
 */
SX1232_status_t SX1232_get_irq_flags(uint32_t* irq_flags) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t reg_value = 0;
	// Check parameter.
	if (irq_flags == NULL) {
		status = SX1232_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Reset result.
	(*irq_flags) = 0;
	// Read registers.
	status = _SX1232_read_register(SX1232_REG_IRQFLAGS1, &reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	(*irq_flags) |= (reg_value << 8);
	status = _SX1232_read_register(SX1232_REG_IRQFLAGS2, &reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	(*irq_flags) |= (reg_value << 0);
errors:
	return status;
}

/* SELECT SX1232 RF OUTPUT PIN.
 * @param rf_output_pin:	RF output pin to select.
 * @return status:			Function execution status.
 */
SX1232_status_t SX1232_set_rf_output_pin(SX1232_rf_output_pin_t rf_output_pin) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t pa_config_reg_value = 0;
	// Read register.
	status = _SX1232_read_register(SX1232_REG_PACONFIG, &pa_config_reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Compute register.
	switch (rf_output_pin) {
	case SX1232_RF_OUTPUT_PIN_RFO:
		// PaSelect = '0'.
		pa_config_reg_value &= 0x7F;
		sx1232_rf_output_pin = SX1232_RF_OUTPUT_PIN_RFO;
		break;
	case SX1232_RF_OUTPUT_PIN_PABOOST:
		// PaSelect = '1'.
		pa_config_reg_value |= 0x80;
		sx1232_rf_output_pin = SX1232_RF_OUTPUT_PIN_PABOOST;
		break;
	default:
		status = SX1232_ERROR_RF_OUTPUT_PIN;
		goto errors;
	}
	// Program register.
	status = _SX1232_write_register(SX1232_REG_PACONFIG, pa_config_reg_value);
errors:
	return status;
}

/* SET RF OUTPUT POWER.
 * @param rf_output_power_dbm:	RF output power in dBm.
 * @return status:				Function execution status.
 */
SX1232_status_t SX1232_set_rf_output_power(uint8_t rf_output_power_dbm) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t pa_config_reg_value = 0;
	// Read register.
	status = _SX1232_read_register(SX1232_REG_PACONFIG, &pa_config_reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Reset bits 0-3.
	pa_config_reg_value &= 0xF0;
	// Compute register.
	switch (sx1232_rf_output_pin) {
	case SX1232_RF_OUTPUT_PIN_RFO:
		// Check parameter.
		if (rf_output_power_dbm > SX1232_OUTPUT_POWER_RFO_MAX) {
			status = SX1232_ERROR_RF_OUTPUT_POWER_OVERFLOW;
			goto errors;
		}
		if (rf_output_power_dbm < SX1232_OUTPUT_POWER_RFO_MIN) {
			status = SX1232_ERROR_RF_OUTPUT_POWER_UNDERFLOW;
			goto errors;
		}
		// Pout = -1 + OutputPower [dBm].
		pa_config_reg_value |= (rf_output_power_dbm + 1) & 0x0F;
		break;
	case SX1232_RF_OUTPUT_PIN_PABOOST:
		// Check parameter.
		if (rf_output_power_dbm > SX1232_OUTPUT_POWER_PABOOST_MAX) {
			status = SX1232_ERROR_RF_OUTPUT_POWER_OVERFLOW;
			goto errors;
		}
		if (rf_output_power_dbm < SX1232_OUTPUT_POWER_PABOOST_MIN) {
			status = SX1232_ERROR_RF_OUTPUT_POWER_UNDERFLOW;
			goto errors;
		}
		// Pout = 2 + OutputPower [dBm].
		pa_config_reg_value |= (rf_output_power_dbm - 2) & 0x0F;
		break;
	default:
		status = SX1232_ERROR_RF_OUTPUT_PIN;
		goto errors;
	}
	// Program register.
	status = _SX1232_write_register(SX1232_REG_PACONFIG, pa_config_reg_value);
errors:
	return status;
}

/* ENABLE LOW PHASE NOISE PLL.
 * @param:			None.
 * @return status:	Function execution status.
 */
SX1232_status_t SX1232_enable_low_phase_noise_pll(void) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t reg_value = 0;
	// Read register.
	status = _SX1232_read_register(SX1232_REG_PARAMP, &reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Program register.
	status = _SX1232_write_register(SX1232_REG_PARAMP, (reg_value & 0xEF));
errors:
	return status;
}

/* START CONTINUOUS WAVE OUTPUT.
 * @param:			None.
 * @return status:	Function execution status.
 */
SX1232_status_t SX1232_start_cw(void) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// Start data signal.
	GPIO_write(&GPIO_SX1232_DIO2, 1);
	// Start radio.
	status = SX1232_set_mode(SX1232_MODE_FSTX);
	if (status != SX1232_SUCCESS) goto errors;
	// Wait TS_FS=60us typical.
	lptim1_status = LPTIM1_delay_milliseconds(2, LPTIM_DELAY_MODE_STOP);
	LPTIM1_status_check(SX1232_ERROR_BASE_LPTIM);
	// TX mode.
	status = SX1232_set_mode(SX1232_MODE_TX);
	if (status != SX1232_SUCCESS) goto errors;
	// Wait TS_TR=120us typical.
	lptim1_status = LPTIM1_delay_milliseconds(2, LPTIM_DELAY_MODE_STOP);
	LPTIM1_status_check(SX1232_ERROR_BASE_LPTIM);
errors:
	return status;
}

/* STOP CONTINUOUS WAVE OUTPUT.
 * @param:			None.
 * @return status:	Function execution status.
 */
SX1232_status_t SX1232_stop_cw(void) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// Stop data signal.
	GPIO_write(&GPIO_SX1232_DIO2, 0);
	// Wait ramp down.
	lptim1_status = LPTIM1_delay_milliseconds(2, LPTIM_DELAY_MODE_STOP);
	LPTIM1_status_check(SX1232_ERROR_BASE_LPTIM);
	// Stop radio.
	status = SX1232_set_mode(SX1232_MODE_STANDBY);
errors:
	return status;
}

/* SET SX1232 RX BANDWIDTH.
 * @param rxbw_mantissa:	RXBW mantissa (see p.30 of SX1232 datasheet).
 * @param rxbw_exponent:	RXBW exponent (see p.30 of SX1232 datasheet).
 * @return status:			Function execution status.
 */
SX1232_status_t SX1232_set_rx_bandwidth(SX1232_rxbw_mantissa_t rxbw_mantissa, SX1232_rxbw_exponent_t rxbw_exponent) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t rxbw_reg_value = 0;
	// Check parameters.
	if (rxbw_mantissa >= SX1232_RXBW_MANTISSA_LAST) {
		status = SX1232_ERROR_RXBW_MANTISSA;
		goto errors;
	}
	if (rxbw_exponent >= SX1232_RXBW_EXPONENT_LAST) {
		status = SX1232_ERROR_RXBW_EXPONENT;
		goto errors;
	}
	// Read register.
	status = _SX1232_read_register(SX1232_REG_RXBW, &rxbw_reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Compute register.
	rxbw_reg_value &= 0xE0; // Reset bits 0-4.
	rxbw_reg_value |= ((rxbw_mantissa & 0x03) << 3) | (rxbw_exponent & 0x02);
	// Program register.
	status = _SX1232_write_register(SX1232_REG_RXBW, rxbw_reg_value);
errors:
	return status;
}

/* CONTROL SX1232 LNA BOOST.
 * @param lna_boost_enable:	Enable (1) or disable (0) LNA boost.
 * @return status:			Function execution status.
 */
SX1232_status_t SX1232_enable_lna_boost(uint8_t lna_boost_enable) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t lna_reg_value = 0;
	// Read register.
	status = _SX1232_read_register(SX1232_REG_LNA, &lna_reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Compute register.
	if (lna_boost_enable != 0) {
		lna_reg_value |= 0x03; // LnaBoost = '11'.
	}
	else {
		lna_reg_value &= 0xFC; // LnaBoost = '00'.
	}
	// Program register.
	status = _SX1232_write_register(SX1232_REG_LNA, lna_reg_value);
errors:
	return status;
}

/* SET PREAMBLE DETECTOR.
 * @param preamble_length_bytes:	Preamble length in bytes (0 disables preamble detector).
 * @param preamble_polarity:		Use 0xAA (0) or 0x55 (1) as preamble byte.
 * @return status:					Function execution status.
 */
SX1232_status_t SX1232_set_preamble_detector(uint8_t preamble_length_bytes, uint8_t preamble_polarity) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t reg_value = 0;
	// Check parameter.
	if (preamble_length_bytes > SX1232_PREAMBLE_LENGTH_BYTES_MAX) {
		status = SX1232_ERROR_PREAMBLE_LENGTH;
		goto errors;
	}
	// Read register.
	status = _SX1232_read_register(SX1232_REG_PREAMBLEDETECT, &reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Check length;
	if (preamble_length_bytes == 0) {
		reg_value &= 0x7F; // Disable preamble detector.
	}
	else {
		reg_value &= 0x9F; // Reset bits 5-6.
		reg_value |= (preamble_length_bytes - 1);
		reg_value |= 0x80; // Enable preamble detector.
	}
	// Program register.
	status = _SX1232_write_register(SX1232_REG_PREAMBLEDETECT, reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Set polarity.
	status = _SX1232_read_register(SX1232_REG_SYNCCONFIG, &reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Program register.
	if (preamble_polarity == 0) {
		status = _SX1232_write_register(SX1232_REG_SYNCCONFIG, (reg_value & 0xDF));
	}
	else {
		status = _SX1232_write_register(SX1232_REG_SYNCCONFIG, (reg_value | 0x20));
	}
errors:
	return status;
}

/* CONFIGURE RX SYNCHRONIZATION WORD.
 * @param sync_word:				Synchronization word.
 * @param sync_word_length_bytes:	Synchronization word length in bytes.
 * @return status:					Function execution status.
 */
SX1232_status_t SX1232_set_sync_word(uint8_t* sync_word, uint8_t sync_word_length_bytes) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t reg_value = 0;
	uint8_t idx = 0;
	// Check parameters.
	if (sync_word == NULL) {
		status = SX1232_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if (sync_word_length_bytes > SX1232_SYNC_WORD_MAXIMUM_LENGTH_BYTES) {
		status = SX1232_ERROR_SYNC_WORD_LENGTH;
		goto errors;
	}
	// Read register.
	status = _SX1232_read_register(SX1232_REG_SYNCCONFIG, &reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Compute register.
	reg_value &= 0xF8; // Reset bits 0-2.
	reg_value |= ((sync_word_length_bytes - 1) & 0x07);
	reg_value &= 0x3F; // Disable receiver auto_restart.
	reg_value |= 0x10; // Enable syncronization word detector.
	// Program register.
	status = _SX1232_write_register(SX1232_REG_SYNCCONFIG, reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Fill synchronization word.
	for (idx=0 ; idx<sync_word_length_bytes ; idx++) {
		// Warning: this loop is working because registers addresses are adjacent.
		status = _SX1232_write_register((SX1232_REG_SYNCVALUE1 + idx), sync_word[idx]);
		if (status != SX1232_SUCCESS) goto errors;
	}
errors:
	return status;
}

/* SET RX PAYLOAD LENGTH.
 * @param data_length_bytes:	Data length to receive in bytes.
 * @return status:				Function execution status.
 */
SX1232_status_t SX1232_set_data_length(uint8_t data_length_bytes) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	// Use fixed length, no CRC computation, do not clear FIFO when CRC fails.
	status = _SX1232_write_register(SX1232_REG_PACKETCONFIG1, 0x08);
	if (status != SX1232_SUCCESS) goto errors;
	// Set data length.
	status = _SX1232_write_register(SX1232_REG_PAYLOADLENGTH, data_length_bytes);
errors:
	return status;
}

/* CONFIGURE SX1232 RSSI MEASUREMENT.
 * @param rssi_sampling:	Number of samples used to average RSSI value (see SX1232_rssi_sampling_t enumeration in sx1232.h).
 * @return status:			Function execution status.
 */
SX1232_status_t SX1232_configure_rssi(SX1232_rssi_sampling_t rssi_sampling) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	// Check parameter.
	if (rssi_sampling >= SX1232_RSSI_SAMPLING_LAST) {
		status = SX1232_ERROR_RSSI_SAMPLING;
		goto errors;
	}
	// Program register
	status = _SX1232_write_register(SX1232_REG_RSSICONFIG, rssi_sampling);
errors:
	return status;
}

/* READ SX1232 RSSI.
 * @param rssi_dbm:	Pointer to 32-bits value that will contain absolute RSSI value in dBm.
 * @return status:	Function execution status.
 */
SX1232_status_t SX1232_get_rssi(int16_t* rssi_dbm) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t rssi_reg_value = 0;
	// Check parameter.
	if (rssi_dbm == NULL) {
		status = SX1232_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Read RSSI register.
	status = _SX1232_read_register(SX1232_REG_RSSIVALUE, &rssi_reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Compute RSSI.
	(*rssi_dbm) = (-1) * (((int16_t) rssi_reg_value / 2) + SX1232_RSSI_OFFSET_DB);
errors:
	return status;
}

/* READ SX1232 FIFO.
 * @param fifo_data:		Byte array that will contain FIFO data.
 * @param fifo_data_length:	Number of bytes to read in FIFO.
 * @return status:			Function execution status.
 */
SX1232_status_t SX1232_read_fifo(uint8_t* fifo_data, uint8_t fifo_data_length) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t idx = 0;
	uint8_t fifo_data_byte = 0;
	// Check parameters.
	if (fifo_data == NULL) {
		status = SX1232_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if (fifo_data_length > SX1232_FIFO_LENGTH) {
		status = SX1232_ERROR_FIFO_LENGTH;
		goto errors;
	}
	// Access FIFO byte per byte.
	for (idx=0 ; idx<fifo_data_length ; idx++) {
		status = _SX1232_read_register(SX1232_REG_FIFO, &fifo_data_byte);
		if (status != SX1232_SUCCESS) goto errors;
		fifo_data[idx] = fifo_data_byte;
	}
errors:
	return status;
}
