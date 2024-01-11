/*
 * spi.c
 *
 *  Created on: 19 jun. 2018
 *      Author: Ludo
 */

#include "spi.h"

#include "gpio.h"
#include "mapping.h"
#include "rcc_reg.h"
#include "spi_reg.h"
#include "types.h"

/*** SPI local macros ***/

#define SPI_ACCESS_TIMEOUT_COUNT	1000000

/*** SPI functions ***/

/*******************************************************************/
void SPI1_init(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 12); // SPI1EN='1'.
	// Configure peripheral.
	// Master mode (MSTR='1').
	// Baud rate = PCLK2/2 = SYSCLK/2 = 8MHz.
	// 16-bits format (DFF='1').
	// Enable output (SSOE='1').
	SPI1 -> CR1 |= (0b1 << 2) | (0b1 << 11);
	SPI1 -> CR2 |= (0b1 << 2);
	// Enable peripheral.
	SPI1 -> CR1 |= (0b1 << 6); // SPE='1'.
	// Configure GPIOs.
	GPIO_configure(&GPIO_SPI1_SCK, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MOSI, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MISO, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
}

/*******************************************************************/
void SPI1_de_init(void) {
	// Disable SPI alternate function.
	GPIO_configure(&GPIO_SPI1_SCK, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MOSI, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MISO, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Disable peripheral.
	SPI1 -> CR1 &= ~(0b1 << 6); // SPE='0'.
	// Disable peripheral clock.
	RCC -> APB2ENR &= ~(0b1 << 12); // SPI1EN='0'.
}

#ifdef HW1_0
/*******************************************************************/
void SPI1_set_clock_configuration(uint8_t prescaler, uint8_t polarity) {
	// Disable peripheral.
	SPI1 -> CR1 &= ~(0b1 << 6); // SPE='0'.
	// Set prescaler.
	SPI1 -> CR1 &= ~(0b111 << 3);
	SPI1 -> CR1 |= ((prescaler & 0x07) << 3);
	// Set polarity.
	if (polarity == 0) {
		SPI1 -> CR1 &= ~(0b11 << 0); // CPOL='0' and CPHA='0'.
	}
	else {
		SPI1 -> CR1 |= (0b11 << 0); // CPOL='1' and CPHA='1'.
	}
	// Enable peripheral.
	SPI1 -> CR1 |= (0b1 << 6); // SPE='1'.
}
#endif

/*******************************************************************/
SPI_status_t SPI1_write_read(uint16_t* tx_data, uint16_t* rx_data, uint8_t transfer_size) {
	// Local variables.
	SPI_status_t status = SPI_SUCCESS;
	uint8_t transfer_idx = 0;
	uint32_t loop_count = 0;
	// Check parameters.
	if ((tx_data == NULL) || (rx_data == NULL)) {
		status = SPI_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Transfer loop.
	for (transfer_idx=0 ; transfer_idx<transfer_size ; transfer_idx++) {
		// Dummy read to DR to clear RXNE flag.
		rx_data[transfer_idx] = *((volatile uint16_t*) &(SPI1 -> DR));
		// Wait for TXE flag.
		while (((SPI1 -> SR) & (0b1 << 1)) == 0) {
			// Wait for TXE='1' or timeout.
			loop_count++;
			if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) {
				status = SPI_ERROR_TX_BUFFER_EMPTY;
				goto errors;
			}
		}
		// Send TX byte.
		*((volatile uint16_t*) &(SPI1 -> DR)) = tx_data[transfer_idx];
		// Wait for incoming data.
		loop_count = 0;
		while (((SPI1 -> SR) & (0b1 << 0)) == 0) {
			// Wait for RXNE='1' or timeout.
			loop_count++;
			if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) {
				status = SPI_ERROR_RX_TIMEOUT;
				goto errors;
			}
		}
		rx_data[transfer_idx] = *((volatile uint16_t*) &(SPI1 -> DR));
	}
errors:
	return status;
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) SPI1_write(uint16_t tx_data) {
	// Send TX byte.
	*((volatile uint16_t*) &(SPI1 -> DR)) = tx_data;
	// Wait for TXE flag.
	while (((SPI1 -> SR) & (0b1 << 1)) == 0);
}

#ifdef HW2_0
/*******************************************************************/
void SPI2_init(void) {
	// Enable peripheral clock.
	RCC -> APB1ENR |= (0b1 << 14); // SPI2EN='1'.
	// Configure peripheral.
	// Master mode (MSTR='1').
	// Baud rate = PCLK2/4 = SYSCLK/4 = 4MHz.
	// 16-bits format (DFF='1').
	// CPOL='1' and CPHA='1'.
	// Enable output (SSOE='1').
	SPI2 -> CR1 |= (0b1 << 2) | (0b001 << 3) | (0b1 << 11) | (0b11 << 0);
	SPI2 -> CR2 |= (0b1 << 2);
	// Enable peripheral.
	SPI2 -> CR1 |= (0b1 << 6); // SPE='1'.
	// Configure GPIOs.
	GPIO_configure(&GPIO_SPI2_SCK, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI2_MOSI, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI2_MISO, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
}
#endif

#ifdef HW2_0
/*******************************************************************/
void SPI2_de_init(void) {
	// Disable SPI alternate function.
	GPIO_configure(&GPIO_SPI2_SCK, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI2_MOSI, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI2_MISO, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Disable peripheral.
	SPI2 -> CR2 &= ~(0b1 << 6); // SPE='0'.
	// Disable peripheral clock.
	RCC -> APB1ENR &= ~(0b1 << 14); // SPI2EN='0'.
}
#endif

#ifdef HW2_0
/*******************************************************************/
SPI_status_t SPI2_write_read(uint16_t* tx_data, uint16_t* rx_data, uint8_t transfer_size) {
	// Local variables.
	SPI_status_t status = SPI_SUCCESS;
	uint8_t transfer_idx = 0;
	uint32_t loop_count = 0;
	// Check parameters.
	if ((tx_data == NULL) || (rx_data == NULL)) {
		status = SPI_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Transfer loop.
	for (transfer_idx=0 ; transfer_idx<transfer_size ; transfer_idx++) {
		// Dummy read to DR to clear RXNE flag.
		rx_data[transfer_idx] = *((volatile uint16_t*) &(SPI2 -> DR));
		// Wait for TXE flag.
		while (((SPI2 -> SR) & (0b1 << 1)) == 0) {
			// Wait for TXE='1' or timeout.
			loop_count++;
			if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) {
				status = SPI_ERROR_TX_BUFFER_EMPTY;
				goto errors;
			}
		}
		// Send TX byte.
		*((volatile uint16_t*) &(SPI2 -> DR)) = tx_data[transfer_idx];
		// Wait for incoming data.
		loop_count = 0;
		while (((SPI2 -> SR) & (0b1 << 0)) == 0) {
			// Wait for RXNE='1' or timeout.
			loop_count++;
			if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) {
				status = SPI_ERROR_RX_TIMEOUT;
				goto errors;
			}
		}
		rx_data[transfer_idx] = *((volatile uint16_t*) &(SPI2 -> DR));
	}
errors:
	return status;
}
#endif
