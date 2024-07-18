/*
 * sx1232.c
 *
 *  Created on: 20 jun. 2018
 *      Author: Ludo
 */

#include "sx1232.h"

#include "error.h"
#include "error_base.h"
#include "gpio.h"
#include "lptim.h"
#include "mapping.h"
#include "spi.h"
#include "sx1232_reg.h"
#include "tim.h"
#include "types.h"

/*** SX1232 local macros ***/

// SPI transfer.
#define SX1232_REGISTER_SPI_TRANSFER_SIZE		2
// RF frequency range.
#define SX1232_RF_FREQUENCY_HZ_MIN				862000000
#define SX1232_RF_FREQUENCY_HZ_MAX				1020000000
// Oscillator frequency.
#define SX1232_FXOSC_HZ							32000000
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
// SX1232 maximum preamble size.
#define SX1232_PREAMBLE_LENGTH_BYTES_MAX		3
// SX1232 DIOs.
#define SX1232_DIO_NUMBER						6
#define SX1232_DIO_MAPPING_MAX_VALUE			4
// SX1232 FIFO.
#define SX1232_FIFO_SIZE_BYTES					64

/*** SX1232 local structures ***/

/*******************************************************************/
typedef struct {
	SX1232_rf_output_pin_t rf_output_pin;
	uint16_t spi_tx_data;
	uint16_t spi_rx_data;
	uint16_t spi_tx_pa_power_value;
} SX1232_context_t;

/*** SX1232 local global variables ***/

static SX1232_context_t sx1232_ctx;

/*** SX1232 local functions ***/

/*******************************************************************/
SX1232_status_t _SX1232_write_register(uint8_t addr, uint8_t value) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	SPI_status_t spi1_status = SPI_SUCCESS;
	// Check parameters.
	if (addr > SX1232_REG_LAST) {
		status = SX1232_ERROR_REGISTER_ADDRESS;
		goto errors;
	}
	// Build SPI frame.
	sx1232_ctx.spi_tx_data = ((addr | 0x80) << 8);
	sx1232_ctx.spi_tx_data |= (value << 0);
	// Write access sequence.
	GPIO_SX1232_CS_LOW();
	spi1_status = SPI1_write_read(&sx1232_ctx.spi_tx_data, &sx1232_ctx.spi_rx_data, 1);
	SPI1_exit_error(SX1232_ERROR_BASE_SPI1);
errors:
	GPIO_SX1232_CS_HIGH();
	return status;
}

/*******************************************************************/
SX1232_status_t _SX1232_read_register(uint8_t addr, uint8_t* value) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	SPI_status_t spi1_status = SPI_SUCCESS;
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
	sx1232_ctx.spi_tx_data = ((addr & 0x7F) << 8);
	// Read access sequence.
	GPIO_SX1232_CS_LOW();
	spi1_status = SPI1_write_read(&sx1232_ctx.spi_tx_data, &sx1232_ctx.spi_rx_data, 1);
	SPI1_exit_error(SX1232_ERROR_BASE_SPI1);
	// Update value.
	(*value) = (uint8_t) (sx1232_ctx.spi_rx_data & 0x00FF);
errors:
	GPIO_SX1232_CS_HIGH();
	return status;
}

/*** SX1232 functions ***/

/*******************************************************************/
void SX1232_init(void) {
	// Init context.
	sx1232_ctx.rf_output_pin = SX1232_RF_OUTPUT_PIN_RFO;
	sx1232_ctx.spi_tx_pa_power_value = ((SX1232_REG_PAVALUE | 0x80) << 8);
	// Init SPI.
	SPI1_init();
	// Configure chip select pin.
	GPIO_configure(&GPIO_SX1232_CS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_write(&GPIO_SX1232_CS, 1);
	// Init SX1232 DIOx.
	GPIO_configure(&GPIO_SX1232_DIO0, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SX1232_DIO2, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/*******************************************************************/
void SX1232_de_init(void) {
	// Release chip select pin.
	GPIO_write(&GPIO_SX1232_CS, 0);
	// Release SX1232 DIOx.
	GPIO_configure(&GPIO_SX1232_DIO0, GPIO_MODE_OUTPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_write(&GPIO_SX1232_DIO2, 0);
	// Release SPI.
	SPI1_de_init();
}

/*******************************************************************/
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
	lptim1_status = LPTIM1_delay_milliseconds(SX1232_OSCILLATOR_DELAY_MS, LPTIM_DELAY_MODE_SLEEP);
	LPTIM1_exit_error(SX1232_ERROR_BASE_LPTIM1);
	// Trigger RC oscillator calibration.
	status = _SX1232_write_register(SX1232_REG_OSC, 0x0F);
errors:
	return status;
}

/*******************************************************************/
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
		// Already done by previous reset.
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
	// Program register.
	status = _SX1232_write_register(SX1232_REG_OPMODE, op_mode_reg_value);
errors:
	return status;
}

/*******************************************************************/
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
		// Already done by previous reset.
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
		// Already done by previous reset.
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
	// Program register.
	status = _SX1232_write_register(SX1232_REG_OPMODE, op_mode_reg_value);
errors:
	return status;
}

/*******************************************************************/
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
	frf_reg_value = (((uint64_t) rf_frequency_hz) << 19) / ((uint64_t) SX1232_FXOSC_HZ);
	// Program register.
	status = _SX1232_write_register(SX1232_REG_FRFMSB, (uint8_t) (frf_reg_value >> 16));
	if (status != SX1232_SUCCESS) goto errors;
	status = _SX1232_write_register(SX1232_REG_FRFMID, (uint8_t) (frf_reg_value >> 8));
	if (status != SX1232_SUCCESS) goto errors;
	status = _SX1232_write_register(SX1232_REG_FRFLSB, (uint8_t) (frf_reg_value >> 0));
errors:
	return status;
}

/*******************************************************************/
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

/*******************************************************************/
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
	fdev_reg_value = (((uint64_t) fsk_deviation_hz) << 19) / ((uint64_t) SX1232_FXOSC_HZ);
	// Program register.
	status = _SX1232_write_register(SX1232_REG_FDEVMSB, (uint8_t) (fdev_reg_value >> 8));
	if (status != SX1232_SUCCESS) goto errors;
	status = _SX1232_write_register(SX1232_REG_FDEVLSB, (uint8_t) (fdev_reg_value >> 0));
errors:
	return status;
}

/*******************************************************************/
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
	bit_rate_reg_value = (SX1232_FXOSC_HZ / bit_rate_bps);
	// Program register.
	status = _SX1232_write_register(SX1232_REG_BITRATEFRAC, 0x00);
	if (status != SX1232_SUCCESS) goto errors;
	status = _SX1232_write_register(SX1232_REG_BITRATEMSB, (uint8_t) (bit_rate_reg_value >> 8));
	if (status != SX1232_SUCCESS) goto errors;
	status = _SX1232_write_register(SX1232_REG_BITRATELSB, (uint8_t) (bit_rate_reg_value >> 0));
errors:
	return status;
}

/*******************************************************************/
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
	case SX1232_DATA_MODE_CONTINUOUS:
		// Already done by previous reset.
		break;
	case SX1232_DATA_MODE_PACKET:
		// Data mode = '1'.
		packet_config2_reg_value |= 0x40;
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

/*******************************************************************/
SX1232_status_t SX1232_set_data_size(uint8_t data_size_bytes) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t packet_config2_reg_value = 0;
	// Use fixed size, no CRC computation, do not clear FIFO when CRC fails.
	status = _SX1232_write_register(SX1232_REG_PACKETCONFIG1, 0x08);
	if (status != SX1232_SUCCESS) goto errors;
	// Set data size.
	status = _SX1232_read_register(SX1232_REG_PACKETCONFIG2, &packet_config2_reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Reset MSB.
	packet_config2_reg_value &= 0xF8;
	// Set data size.
	status = _SX1232_write_register(SX1232_REG_PACKETCONFIG2, packet_config2_reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	status = _SX1232_write_register(SX1232_REG_PAYLOADLENGTH, data_size_bytes);
errors:
	return status;
}

/*******************************************************************/
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
	dio_mapping_reg_addr = SX1232_REG_DIOMAPPING1 + (dio >> 2);
	// Read register.
	status = _SX1232_read_register(dio_mapping_reg_addr, &dio_mapping_reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Compute register.
	dio_mapping_reg_value &= ~(0b11 << ((dio % 4) << 1));
	dio_mapping_reg_value |= (dio_mapping << ((dio % 4) << 1));
	// Write register.
	status = _SX1232_write_register(dio_mapping_reg_addr, dio_mapping_reg_value);
errors:
	return status;
}

/*******************************************************************/
SX1232_status_t SX1232_get_irq_flag(SX1232_irq_index_t irq_index, uint8_t* irq_flag) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t reg_value = 0;
	uint8_t reg_addr_offset = 0;
	uint8_t irq_bit_offset = 0;
	// Check parameters.
	if (irq_index >= SX1232_IRQ_INDEX_LAST) {
		status = SX1232_ERROR_IRQ_INDEX;
		goto errors;
	}
	if (irq_flag == NULL) {
		status = SX1232_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Get register and bit offsets.
	reg_addr_offset = (irq_index >> 3);
	irq_bit_offset = (irq_index % 8);
	// Read register.
	status = _SX1232_read_register((SX1232_REG_IRQFLAGS2 - reg_addr_offset), &reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Read bit.
	(*irq_flag) = (reg_value >> irq_bit_offset) & 0x01;
errors:
	return status;
}

/*******************************************************************/
SX1232_status_t SX1232_start_tx(void) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// Start radio.
	status = SX1232_set_mode(SX1232_MODE_FSTX);
	if (status != SX1232_SUCCESS) goto errors;
	// Wait TS_FS=60us typical.
	lptim1_status = LPTIM1_delay_milliseconds(SX1232_STATE_SWITCH_DELAY_MS, LPTIM_DELAY_MODE_SLEEP);
	LPTIM1_exit_error(SX1232_ERROR_BASE_LPTIM1);
	// TX mode.
	status = SX1232_set_mode(SX1232_MODE_TX);
	if (status != SX1232_SUCCESS) goto errors;
	// Wait TS_TR=120us typical.
	lptim1_status = LPTIM1_delay_milliseconds(SX1232_STATE_SWITCH_DELAY_MS, LPTIM_DELAY_MODE_SLEEP);
	LPTIM1_exit_error(SX1232_ERROR_BASE_LPTIM1);
errors:
	return status;
}

/*******************************************************************/
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
		sx1232_ctx.rf_output_pin = SX1232_RF_OUTPUT_PIN_RFO;
		break;
	case SX1232_RF_OUTPUT_PIN_PABOOST:
		// PaSelect = '1'.
		pa_config_reg_value |= 0x80;
		sx1232_ctx.rf_output_pin = SX1232_RF_OUTPUT_PIN_PABOOST;
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

/*******************************************************************/
SX1232_status_t SX1232_set_rf_output_power(int8_t rf_output_power_dbm) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t pa_config_reg_value = 0;
	// Read register.
	status = _SX1232_read_register(SX1232_REG_PACONFIG, &pa_config_reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Reset bits 0-3.
	pa_config_reg_value &= 0xF0;
	// Compute register.
	switch (sx1232_ctx.rf_output_pin) {
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

/*******************************************************************/
SX1232_status_t SX1232_enable_manual_pa_control(void) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	// Set required registers.
	status = _SX1232_write_register(SX1232_REG_PLLHOP, 0x7B);
	if (status != SX1232_SUCCESS) goto errors;
	status = _SX1232_write_register(0x4D, 0x03);
	if (status != SX1232_SUCCESS) goto errors;
	status = _SX1232_write_register(SX1232_REG_PAMANUAL, 0x60);
	if (status != SX1232_SUCCESS) goto errors;
errors:
	return status;
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) SX1232_set_pa_power_value(uint8_t pa_power_value) {
	// Build SPI frame.
	sx1232_ctx.spi_tx_pa_power_value &= 0xFF00;
	sx1232_ctx.spi_tx_pa_power_value |= pa_power_value;
	// Write access sequence.
	GPIO_SX1232_CS_LOW();
	SPI1_write(sx1232_ctx.spi_tx_pa_power_value);
	GPIO_SX1232_CS_HIGH();
}

/*******************************************************************/
SX1232_status_t SX1232_start_rx(void) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// Start radio.
	status = SX1232_set_mode(SX1232_MODE_FSRX);
	if (status != SX1232_SUCCESS) goto errors;
	// Wait TS_FS=60us typical.
	lptim1_status = LPTIM1_delay_milliseconds(SX1232_STATE_SWITCH_DELAY_MS, LPTIM_DELAY_MODE_SLEEP);
	LPTIM1_exit_error(SX1232_ERROR_BASE_LPTIM1);
	// TX mode.
	status = SX1232_set_mode(SX1232_MODE_RX);
	if (status != SX1232_SUCCESS) goto errors;
	// Wait TS_TR=120us typical.
	lptim1_status = LPTIM1_delay_milliseconds(SX1232_STATE_SWITCH_DELAY_MS, LPTIM_DELAY_MODE_SLEEP);
	LPTIM1_exit_error(SX1232_ERROR_BASE_LPTIM1);
errors:
	return status;
}

/*******************************************************************/
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
	rxbw_reg_value |= ((rxbw_mantissa & 0x03) << 3) | (rxbw_exponent & 0x07);
	// Program register.
	status = _SX1232_write_register(SX1232_REG_RXBW, rxbw_reg_value);
errors:
	return status;
}

/*******************************************************************/
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

/*******************************************************************/
SX1232_status_t SX1232_set_preamble_detector(uint8_t preamble_size_bytes, uint8_t preamble_polarity) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t reg_value = 0;
	// Check parameter.
	if (preamble_size_bytes > SX1232_PREAMBLE_LENGTH_BYTES_MAX) {
		status = SX1232_ERROR_PREAMBLE_LENGTH;
		goto errors;
	}
	// Read register.
	status = _SX1232_read_register(SX1232_REG_PREAMBLEDETECT, &reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Check size;
	if (preamble_size_bytes == 0) {
		reg_value &= 0x7F; // Disable preamble detector.
	}
	else {
		reg_value &= 0x9F; // Reset bits 5-6.
		reg_value |= (preamble_size_bytes - 1);
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
		reg_value &= 0xDF;
	}
	else {
		reg_value |= 0x20;
	}
	status = _SX1232_write_register(SX1232_REG_SYNCCONFIG, reg_value);
errors:
	return status;
}

/*******************************************************************/
SX1232_status_t SX1232_set_sync_word(uint8_t* sync_word, uint8_t sync_word_size_bytes) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t reg_value = 0;
	uint8_t idx = 0;
	// Check parameters.
	if (sync_word == NULL) {
		status = SX1232_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if (sync_word_size_bytes > SX1232_SYNC_WORD_MAXIMUM_LENGTH_BYTES) {
		status = SX1232_ERROR_SYNC_WORD_LENGTH;
		goto errors;
	}
	// Read register.
	status = _SX1232_read_register(SX1232_REG_SYNCCONFIG, &reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Compute register.
	reg_value &= 0x38; // Reset bits 0-2 and disable receiver auto_restart.
	reg_value |= ((sync_word_size_bytes - 1) & 0x07);
	reg_value |= 0x10; // Enable synchronization word detector.
	// Program register.
	status = _SX1232_write_register(SX1232_REG_SYNCCONFIG, reg_value);
	if (status != SX1232_SUCCESS) goto errors;
	// Fill synchronization word.
	for (idx=0 ; idx<sync_word_size_bytes ; idx++) {
		// Warning: this loop is working because registers addresses are adjacent.
		status = _SX1232_write_register((SX1232_REG_SYNCVALUE1 + idx), sync_word[idx]);
		if (status != SX1232_SUCCESS) goto errors;
	}
errors:
	return status;
}

/*******************************************************************/
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

/*******************************************************************/
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

/*******************************************************************/
SX1232_status_t SX1232_read_fifo(uint8_t* fifo_data, uint8_t fifo_data_size) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	uint8_t idx = 0;
	uint8_t fifo_data_byte = 0;
	// Check parameters.
	if (fifo_data == NULL) {
		status = SX1232_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if (fifo_data_size > SX1232_FIFO_SIZE_BYTES) {
		status = SX1232_ERROR_FIFO_LENGTH;
		goto errors;
	}
	// Access FIFO byte per byte.
	for (idx=0 ; idx<fifo_data_size ; idx++) {
		status = _SX1232_read_register(SX1232_REG_FIFO, &fifo_data_byte);
		if (status != SX1232_SUCCESS) goto errors;
		fifo_data[idx] = fifo_data_byte;
	}
errors:
	return status;
}
