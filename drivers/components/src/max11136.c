/*
 * max11136.c
 *
 *  Created on: 17 nov. 2018
 *      Author: Ludo
 */

#include "max11136.h"

#include "gpio.h"
#include "gpio_mapping.h"
#include "lptim.h"
#include "mode.h"
#include "spi.h"
#include "types.h"

/*** MAX11136 local macros ***/

#ifdef HW1_0
#define MAX11136_SPI_INSTANCE			SPI_INSTANCE_SPI1
#endif
#ifdef HW2_0
#define MAX11136_SPI_INSTANCE			SPI_INSTANCE_SPI2
#endif

#define MAX11136_SUB_DELAY_MS			100
#define MAX11136_TIMEOUT_MS				2000

/*** MAX11136 local structures ***/

/*******************************************************************/
typedef enum {
	MAX11136_REGISTER_ADC_MODE_CONTROL = 0b00000,
	MAX11136_REGISTER_ADC_CONFIG = 0b10000,
	MAX11136_REGISTER_UNIPOLAR = 0b10001,
	MAX11136_REGISTER_BIPOLAR = 0b10010,
	MAX11136_REGISTER_RANGE = 0b10011,
	MAX11136_REGISTER_CUSTOM_SCAN0 = 0b10100,
	MAX11136_REGISTER_CUSTOM_SCAN1 = 0b10101,
	MAX11136_REGISTER_SAMPLE_SET = 0b10110,
	MAX11136_REGISTER_LAST = 0b10111
} MAX11136_register_t;

/*** MAX11136 local functions ***/

/*******************************************************************/
static MAX11136_status_t _MAX11136_write_register(MAX11136_register_t register_address, uint16_t value) {
	// Local variables.
	MAX11136_status_t status = MAX11136_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	uint16_t spi_command = 0;
	uint16_t spi_reply = 0;
	// Check parameters.
	if (register_address >= MAX11136_REGISTER_LAST) {
		status = MAX11136_ERROR_REGISTER_ADDRESS;
		goto errors;
	}
	// Build SPI command.
	if (register_address == MAX11136_REGISTER_ADC_MODE_CONTROL) {
		// Data is 15-bits length.
		spi_command |= value & 0x00007FFF;
	}
	else {
		// Data is 11-bits length.
		spi_command |= (register_address & 0x0000001F) << 11;
		spi_command |= (value & 0x000007FF);
	}
	// CS low.
	GPIO_write(&GPIO_MAX11136_CS, 0);
	// SPI transfer.
	spi_status = SPI_write_read_16(MAX11136_SPI_INSTANCE, &spi_command, &spi_reply, 1);
	SPI_exit_error(MAX11136_ERROR_BASE_SPI);
errors:
	// CS high.
	GPIO_write(&GPIO_MAX11136_CS, 1);
	return status;
}

/*** MAX11136 functions ***/

/*******************************************************************/
MAX11136_status_t MAX11136_init(void) {
	// Local variables.
	MAX11136_status_t status = MAX11136_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	SPI_configuration_t spi_config;
	// Init SPI.
	spi_config.baud_rate_prescaler = SPI_BAUD_RATE_PRESCALER_4;
	spi_config.data_format = SPI_DATA_FORMAT_16_BITS;
	spi_config.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
	spi_status = SPI_init(MAX11136_SPI_INSTANCE, &GPIO_MAX11136_SPI, &spi_config);
	SPI_exit_error(MAX11136_ERROR_BASE_SPI);
	// Configure chip select pin.
	GPIO_configure(&GPIO_MAX11136_CS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_write(&GPIO_MAX11136_CS, 1);
	// Configure EOC GPIO.
	GPIO_configure(&GPIO_MAX11136_EOC, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_UP);
errors:
	return status;
}

/*******************************************************************/
MAX11136_status_t MAX11136_de_init(void) {
	// Local variables.
	MAX11136_status_t status = MAX11136_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	// Release chip select pin.
	GPIO_write(&GPIO_MAX11136_CS, 0);
	// Release EOC GPIO.
	GPIO_configure(&GPIO_MAX11136_EOC, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Release SPI.
	spi_status = SPI_de_init(MAX11136_SPI_INSTANCE, &GPIO_MAX11136_SPI);
	SPI_exit_error(MAX11136_ERROR_BASE_SPI);
errors:
	return status;
}

/*******************************************************************/
MAX11136_status_t MAX11136_convert_channel(MAX11136_channel_t channel, uint16_t* adc_data_12bits) {
	// Local variables.
	MAX11136_status_t status = MAX11136_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	LPTIM_status_t lptim_status = LPTIM_SUCCESS;
	uint16_t spi_command = 0;
	uint16_t spi_data = 0;
	uint32_t loop_count_ms = 0;
	// Check parameters.
	if (channel >= MAX11136_CHANNEL_LAST) {
		status = MAX11136_ERROR_CHANNEL;
		goto errors;
	}
	if (adc_data_12bits == NULL) {
		status = MAX11136_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Configure ADC.
	// Single-ended unipolar (already done at POR).
	// Enable averaging: AVGON='1' and NAVG='00' (4 conversions).
	status = _MAX11136_write_register(MAX11136_REGISTER_ADC_CONFIG, 0x0200);
	if (status != MAX11136_SUCCESS) goto errors;
	// Select channel.
	status = _MAX11136_write_register(MAX11136_REGISTER_CUSTOM_SCAN1, (0b1 << (channel + 3)));
	if (status != MAX11136_SUCCESS) goto errors;
	// Scan mode = custom internal: SCAN='0111'.
	// Reset the FIFO: RESET='01'.
	// Auto shutdown: PM='01'.
	// Start conversion: SWCNV='1'.
	status = _MAX11136_write_register(MAX11136_REGISTER_ADC_MODE_CONTROL, 0x382A);
	if (status != MAX11136_SUCCESS) goto errors;
	// Wait for EOC to be pulled low.
	while (GPIO_read(&GPIO_MAX11136_EOC) != 0) {
		// Low power delay.
		lptim_status = LPTIM_delay_milliseconds(MAX11136_SUB_DELAY_MS, LPTIM_DELAY_MODE_STOP);
		LPTIM_exit_error(MAX11136_ERROR_BASE_LPTIM);
		// Exit if timeout.
		loop_count_ms += MAX11136_SUB_DELAY_MS;
		if (loop_count_ms > MAX11136_TIMEOUT_MS) {
			status = MAX11136_ERROR_TIMEOUT;
			goto errors;
		}
	}
	// CS low.
	GPIO_write(&GPIO_MAX11136_CS, 0); // Falling edge on CS pin.
	// SPI transfer.
	spi_status = SPI_write_read_16(MAX11136_SPI_INSTANCE, &spi_command, &spi_data, 1);
	SPI_exit_error(MAX11136_ERROR_BASE_SPI);
	// Check output data.
	if ((spi_data >> 12) != channel) {
		status = MAX11136_ERROR_OUTPUT_CHANNEL;
		goto errors;
	}
	// Parse output data.
	(*adc_data_12bits) = (spi_data & 0x0FFF);
errors:
	GPIO_write(&GPIO_MAX11136_CS, 1);
	return status;
}
