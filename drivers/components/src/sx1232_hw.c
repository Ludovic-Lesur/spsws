/*
 * max11136.c
 *
 *  Created on: 01 sep. 2024
 *      Author: Ludo
 */

#include "sx1232.h"

#include "error.h"
#include "gpio.h"
#include "gpio_mapping.h"
#include "lptim.h"
#include "spi.h"
#include "types.h"

#ifndef SX1232_DRIVER_DISABLE

/*** SX1232 HW local macros ***/

#define SX1232_SPI_INSTANCE		SPI_INSTANCE_SPI1

/*** SX1232 HW functions ***/

/*******************************************************************/
SX1232_status_t SX1232_HW_init(void) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	SPI_configuration_t spi_config;
	// Init SPI.
	spi_config.baud_rate_prescaler = SPI_BAUD_RATE_PRESCALER_4;
	spi_config.data_format = SPI_DATA_FORMAT_16_BITS;
	spi_config.clock_polarity = SPI_CLOCK_POLARITY_LOW;
	spi_status = SPI_init(SX1232_SPI_INSTANCE, &GPIO_SX1232_SPI, &spi_config);
	SPI_exit_error(SX1232_ERROR_BASE_SPI);
	// Configure chip select pin.
	GPIO_configure(&GPIO_SX1232_CS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_write(&GPIO_SX1232_CS, 1);
errors:
	return status;
}

/*******************************************************************/
SX1232_status_t SX1232_HW_de_init(void) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	// Release chip select pin.
	GPIO_write(&GPIO_SX1232_CS, 0);
	// Release SPI.
	spi_status = SPI_de_init(SX1232_SPI_INSTANCE, &GPIO_SX1232_SPI);
	SPI_exit_error(SX1232_ERROR_BASE_SPI);
errors:
	return status;
}

/*******************************************************************/
SX1232_status_t SX1232_HW_spi_write_read_16(uint16_t* tx_data, uint16_t* rx_data, uint8_t transfer_size) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	// CS low.
	GPIO_write(&GPIO_SX1232_CS, 0);
	// SPI transfer.
	spi_status = SPI_write_read_16(SX1232_SPI_INSTANCE, tx_data, rx_data, transfer_size);
	SPI_exit_error(SX1232_ERROR_BASE_SPI);
errors:
	// CS high.
	GPIO_write(&GPIO_SX1232_CS, 1);
	return status;
}

/*******************************************************************/
void SX1232_HW_spi_write_16(uint16_t tx_data) {
	GPIO_SX1232_CS_LOW();
	SPI_write_16(SX1232_SPI_INSTANCE, tx_data);
	GPIO_SX1232_CS_HIGH();
}

/*******************************************************************/
SX1232_status_t SX1232_HW_delay_milliseconds(uint32_t delay_ms) {
	// Local variables.
	SX1232_status_t status = SX1232_SUCCESS;
	LPTIM_status_t lptim_status = LPTIM_SUCCESS;
	// Perform delay.
	lptim_status = LPTIM_delay_milliseconds(delay_ms, LPTIM_DELAY_MODE_SLEEP);
	LPTIM_exit_error(SX1232_ERROR_BASE_DELAY);
errors:
	return status;
}

#endif /* SX1232_DRIVER_DISABLE */
