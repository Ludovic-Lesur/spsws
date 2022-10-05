/*
 * spi.c
 *
 *  Created on: 19 june 2018
 *      Author: Ludo
 */

#include "spi.h"

#include "gpio.h"
#include "lptim.h"
#include "mapping.h"
#include "rcc_reg.h"
#include "spi_reg.h"
#include "types.h"

/*** SPI local macros ***/

#define SPI_ACCESS_TIMEOUT_COUNT	1000000

/*** SPI functions ***/

/* CONFIGURE SPI1.
 * @param:	None.
 * @return:	None.
 */
void SPI1_init(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 12); // SPI1EN='1'.
	// Configure power enable pins.
	GPIO_configure(&GPIO_RF_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SX1232_CS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#ifdef HW1_0
	GPIO_configure(&GPIO_SENSORS_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_MAX11136_CS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	SPI1_power_off();
	// Configure peripheral.
	// Master mode (MSTR='1').
	// Baud rate = PCLK2/4 = SYSCLK/4 = 4MHz.
	// 8-bits format (DFF='0') by default.
	// Enable output (SSOE='1').
	SPI1 -> CR1 |= (0b1 << 2) | (0b001 << 3);
	SPI1 -> CR2 |= (0b1 << 2);
	// Enable peripheral.
	SPI1 -> CR1 |= (0b1 << 6); // SPE='1'.
}

#ifdef HW1_0
/* SET SPI1 SCLK POLARITY.
 * @param polarity:	Clock polarity (0 = SCLK idle high, otherwise SCLK idle low).
 * @return:			None.
 */
void SPI1_set_clock_polarity(uint8_t polarity) {
	if (polarity == 0) {
		SPI1 -> CR1 &= ~(0b11 << 0); // CPOL='0' and CPHA='0'.
	}
	else {
		SPI1 -> CR1 |= (0b11 << 0); // CPOL='1' and CPHA='1'.
	}
}
#endif

/* SWITCH ALL SPI1 SLAVES ON.
 * @param:			None.
 * @return status:	Function execution status.
 */
SPI_status_t SPI1_power_on(void) {
	// Local variables.
	SPI_status_t status = SPI_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// Enable GPIOs.
	GPIO_configure(&GPIO_SPI1_SCK, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MOSI, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MISO, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	// Turn SPI1 slaves on.
	GPIO_write(&GPIO_RF_POWER_ENABLE, 1);
	// Wait for power-on.
	lptim1_status = LPTIM1_delay_milliseconds(100, 1);
	LPTIM1_status_check(SPI_ERROR_BASE_LPTIM);
#ifdef HW1_0
	GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 1);
	lptim1_status = LPTIM1_delay_milliseconds(100, 1);
	LPTIM1_status_check(SPI_ERROR_BASE_LPTIM);
#endif
	// Chip select high by default.
	GPIO_write(&GPIO_SX1232_CS, 1);
#ifdef HW1_0
	GPIO_write(&GPIO_MAX11136_CS, 1);
	// Add pull-up to EOC.
	GPIO_configure(&GPIO_MAX11136_EOC, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_UP);
#endif
errors:
	return status;
}

/* SWITCH ALL SPI1 SLAVES OFF.
 * @param:	None.
 * @return:	None.
 */
void SPI1_power_off(void) {
	// Turn SPI1 slaves off.
	GPIO_write(&GPIO_RF_POWER_ENABLE, 0);
	GPIO_write(&GPIO_SX1232_CS, 0);
#ifdef HW1_0
	GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 0);
	GPIO_write(&GPIO_MAX11136_CS, 0);
	// Remove pull-up to EOC.
	GPIO_configure(&GPIO_MAX11136_EOC, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	// Disable SPI alternate function.
	GPIO_configure(&GPIO_SPI1_SCK, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MOSI, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI1_MISO, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/* SEND A BYTE THROUGH SPI1.
 * @param tx_data:	8-bits data to send.
 * @return status:	Function execution status.
 */
SPI_status_t SPI1_write_byte(uint8_t tx_data) {
	// Local variables.
	SPI_status_t status = SPI_SUCCESS;
	uint32_t loop_count = 0;
#ifdef HW1_0
	// Set data length to 8-bits.
	SPI1 -> CR1 &= ~(0b1 << 11); // DFF='0'.
#endif
	// Wait for TXE flag.
	while (((SPI1 -> SR) & (0b1 << 1)) == 0) {
		// Wait for TXE='1' or timeout.
		loop_count++;
		if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) {
			status = SPI_ERROR_TX_BUFFER_EMPTY;
			goto errors;
		}
	}
	// Send data.
	*((volatile uint8_t*) &(SPI1 -> DR)) = tx_data;
errors:
	return status;
}

/* READ A BYTE FROM SPI1.
 * @param rx_data:	Pointer to 8-bits value that will contain the data to read.
 * @return status:	Function execution status.
 */
SPI_status_t SPI1_read_byte(uint8_t tx_data, uint8_t* rx_data) {
	// Local variables.
	SPI_status_t status = SPI_SUCCESS;
	uint32_t loop_count = 0;
	// Check parameters.
	if (rx_data == NULL) {
		status = SPI_ERROR_NULL_PARAMETER;
		goto errors;
	}
#ifdef HW1_0
	// Set data length to 8-bits.
	SPI1 -> CR1 &= ~(0b1 << 11); // DFF='0'.
#endif
	// Dummy read to DR to clear RXNE flag.
	(*rx_data) = *((volatile uint8_t*) &(SPI1 -> DR));
	// Wait for TXE flag.
	while (((SPI1 -> SR) & (0b1 << 1)) == 0) {
		// Wait for TXE='1' or timeout.
		loop_count++;
		if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) {
			status = SPI_ERROR_TX_BUFFER_EMPTY;
			goto errors;
		}
	}
	// Send dummy data on MOSI to generate clock.
	*((volatile uint8_t*) &(SPI1 -> DR)) = tx_data;
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
	(*rx_data) = *((volatile uint8_t*) &(SPI1 -> DR));
errors:
	return status;
}

#ifdef HW1_0
/* SEND A SHORT THROUGH SPI1.
 * @param tx_data:	16-bits data to send.
 * @return status:	Function execution status.
 */
SPI_status_t SPI1_write_short(uint16_t tx_data) {
	// Local variables.
	SPI_status_t status = SPI_SUCCESS;
	uint32_t loop_count = 0;
	// Set data length to 16-bits.
	SPI1 -> CR1 |= (0b1 << 11); // DFF='1'.
	// Wait for TXE flag.
	while (((SPI1 -> SR) & (0b1 << 1)) == 0) {
		// Wait for TXE='1' or timeout.
		loop_count++;
		if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) {
			status = SPI_ERROR_TX_BUFFER_EMPTY;
			goto errors;
		}
	}
	// Send data.
	*((volatile uint16_t*) &(SPI1 -> DR)) = tx_data;
errors:
	return status;
}

/* READ A SHORT FROM SPI1.
 * @param rx_data:	Pointer to 16-bits value that will contain the data to read.
 * @return status:	Function execution status.
 */
SPI_status_t SPI1_read_short(uint16_t tx_data, uint16_t* rx_data) {
	// Local variables.
	SPI_status_t status = SPI_SUCCESS;
	uint32_t loop_count = 0;
	// Check parameters.
	if (rx_data == NULL) {
		status = SPI_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Set data length to 16-bits.
	SPI1 -> CR1 |= (0b1 << 11); // DFF='1'.
	// Dummy read to DR to clear RXNE flag.
	(*rx_data) = *((volatile uint16_t*) &(SPI1 -> DR));
	// Wait for TXE flag.
	while (((SPI1 -> SR) & (0b1 << 1)) == 0) {
		// Wait for TXE='1' or timeout.
		loop_count++;
		if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) {
			status = SPI_ERROR_TX_BUFFER_EMPTY;
			goto errors;
		}
	}
	// Send dummy data on MOSI to generate clock.
	*((volatile uint16_t*) &(SPI1 -> DR)) = tx_data;
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
	(*rx_data) = *((volatile uint16_t*) &(SPI1 -> DR));
errors:
	return status;
}
#endif

#ifdef HW2_0
/* CONFIGURE SPI2.
 * @param:	None.
 * @return:	None.
 */
void SPI2_init(void) {
	// Enable peripheral clock.
	RCC -> APB1ENR |= (0b1 << 14); // SPI2EN='1'.
	// Configure power enable pins.
	GPIO_configure(&GPIO_ADC_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_MAX11136_CS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	SPI2_power_off();
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
}

/* SWITCH ALL SPI2 SLAVES ON.
 * @param:			None.
 * @return status:	Function execution status.
 */
SPI_status_t SPI2_power_on(void) {
	// Local variables.
	SPI_status_t status = SPI_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// Enable GPIOs.
	GPIO_configure(&GPIO_SPI2_SCK, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI2_MOSI, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI2_MISO, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Turn MAX11136 on.
	GPIO_write(&GPIO_ADC_POWER_ENABLE, 1);
	// Warm-up delay.
	lptim1_status = LPTIM1_delay_milliseconds(100, 1);
	LPTIM1_status_check(SPI_ERROR_BASE_LPTIM);
	// Chip select high by default.
	GPIO_write(&GPIO_MAX11136_CS, 1);
errors:
	return status;
}

/* SWITCH ALL SPI2 SLAVES OFF.
 * @param:	None.
 * @return:	None.
 */
void SPI2_power_off(void) {
	// Turn MAX11136 off.
	GPIO_write(&GPIO_ADC_POWER_ENABLE, 0);
	GPIO_write(&GPIO_MAX11136_CS, 0);
	// Disable SPI alternate function.
	GPIO_configure(&GPIO_SPI2_SCK, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI2_MOSI, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_SPI2_MISO, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_MAX11136_CS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/* SEND A SHORT THROUGH SPI2.
 * @param tx_data:	Data to send (16-bits).
 * @return status:	Function execution status.
 */
SPI_status_t SPI2_write_short(uint16_t tx_data) {
	// Local variables.
	SPI_status_t status = SPI_SUCCESS;
	uint32_t loop_count = 0;
	// Wait for TXE flag.
	while (((SPI2 -> SR) & (0b1 << 1)) == 0) {
		// Wait for TXE='1' or timeout.
		loop_count++;
		if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) {
			status = SPI_ERROR_TX_BUFFER_EMPTY;
			goto errors;
		}
	}
	// Send data.
	*((volatile uint16_t*) &(SPI2 -> DR)) = tx_data;
errors:
	return status;
}

/* READ A SHORT FROM SPI2.
 * @param rx_data:	Pointer to 16-bits value that will contain the data to read.
 * @return status:	Function execution status.
 */
SPI_status_t SPI2_read_short(uint16_t tx_data, uint16_t* rx_data) {
	// Local variables.
	SPI_status_t status = SPI_SUCCESS;
	uint32_t loop_count = 0;
	// Check parameters.
	if (rx_data == NULL) {
		status = SPI_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Dummy read to DR to clear RXNE flag.
	(*rx_data) = *((volatile uint16_t*) &(SPI2 -> DR));
	// Wait for TXE flag.
	while (((SPI2 -> SR) & (0b1 << 1)) == 0) {
		// Wait for TXE='1' or timeout.
		loop_count++;
		if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) {
			status = SPI_ERROR_TX_BUFFER_EMPTY;
			goto errors;
		}
	}
	// Send dummy data on MOSI to generate clock.
	*((volatile uint16_t*) &(SPI2 -> DR)) = tx_data;
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
	(*rx_data) = *((volatile uint16_t*) &(SPI2 -> DR));
errors:
	return status;
}
#endif
