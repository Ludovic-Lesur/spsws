/*
 * i2c.c
 *
 *  Created on: 12 may 2018
 *      Author: Ludo
 */

#include "i2c.h"

#include "gpio.h"
#include "lptim.h"
#include "mapping.h"
#include "i2c_reg.h"
#include "rcc.h"
#include "rcc_reg.h"

/*** I2C local macros ***/

#define I2C_ACCESS_TIMEOUT_COUNT	1000000

/* CLEAR ALL I2C PERIPHERAL FLAGS
 * @param:			None.
 * @return status:	Function execution status.
 */
static I2C_status_t I2C1_clear(void) {
	// Local variables.
	I2C_status_t status = I2C_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// Disable peripheral.
	I2C1 -> CR1 &= ~(0b1 << 0); // PE='0'.
	lptim1_status = LPTIM1_delay_milliseconds(1, 0);
	LPTIM1_status_check(I2C_ERROR_BASE_LPTIM);
	// Enable peripheral and clear all flags.
	I2C1 -> CR1 |= (0b1 << 0); // PE='1'.
	I2C1 -> ICR |= 0x00003F38;
errors:
	return status;
}

/*** I2C functions ***/

/* CONFIGURE I2C1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void I2C1_init(void) {
	// Enable peripheral clock.
	RCC -> APB1ENR |= (0b1 << 21); // I2C1EN='1'.
	// Configure power enable pin.
	GPIO_configure(&GPIO_SENSORS_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 0);
	// Configure SCL and SDA (first as high impedance).
	GPIO_configure(&GPIO_I2C1_SCL, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_I2C1_SDA, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Configure peripheral.
	I2C1 -> CR1 &= ~(0b1 << 0); // Disable peripheral before configuration (PE='0').
	I2C1 -> CR1 &= ~(0b11111 << 8); // Analog filter enabled (ANFOFF='0') and digital filter disabled (DNF='0000').
	I2C1 -> TIMINGR = 0; // Reset all bits.
	I2C1 -> TIMINGR |= (7 << 28); // I2CCLK = PCLK1/(PRESC+1) = SYSCLK/(PRESC+1) = 2MHz (HSI) (PRESC='1000').
	I2C1 -> TIMINGR |= (99 << 8) + 99; // Set SCL frequency to 10kHz. See p.641 of RM0377 datasheet.
	I2C1 -> CR1 &= ~(0b1 << 17); // Must be kept cleared in master mode (NOSTRETCH='0').
	I2C1 -> CR2 &= ~(0b1 << 11); // 7-bits addressing mode (ADD10='0').
	I2C1 -> CR2 &= ~(0b11 << 24); // AUTOEND='0' and RELOAD='0'.
	// Enable peripheral.
	I2C1 -> CR1 |= (0b1 << 0); // PE='1'.
}

/* DISABLE I2C PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void I2C1_disable(void) {
	// Disable power control pin.
	GPIO_configure(&GPIO_SENSORS_POWER_ENABLE, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Disable I2C1 peripheral.
	I2C1 -> CR1 &= ~(0b1 << 0);
	// Clear all flags.
	I2C1 -> ICR |= 0x00003F38;
	// Disable peripheral clock.
	RCC -> APB1ENR &= ~(0b1 << 21); // I2C1EN='0'.
}

/* SWITCH ALL I2C1 SLAVES ON.
 * @param:			None.
 * @return status:	Function execution status.
 */
I2C_status_t I2C1_power_on(void) {
	// Local variables.
	I2C_status_t status = I2C_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// Enable GPIOs.
	GPIO_configure(&GPIO_I2C1_SCL, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_I2C1_SDA, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Turn sensors and pull-up resistors on.
	GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 1);
	lptim1_status = LPTIM1_delay_milliseconds(100, 1);
	LPTIM1_status_check(I2C_ERROR_BASE_LPTIM);
errors:
	return status;
}

/* SWITCH ALL I2C1 SLAVES ON.
 * @param:			None.
 * @return status:	Function execution status.
 */
I2C_status_t I2C1_power_off(void) {
	// Local variables.
	I2C_status_t status = I2C_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// Turn sensors and pull-up resistors off.
	GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 0);
	// Disable I2C alternate function.
	GPIO_configure(&GPIO_I2C1_SCL, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_I2C1_SDA, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Delay required if another cycle is requested by applicative layer.
	lptim1_status = LPTIM1_delay_milliseconds(100, 1);
	LPTIM1_status_check(I2C_ERROR_BASE_LPTIM);
errors:
	return status;
}

/* WRITE DATA ON I2C1 BUS (see algorithme on p.607 of RM0377 datasheet).
 * @param slave_address:	Slave address on 7 bits.
 * @param tx_buf:			Array containing the byte(s) to send.
 * @param tx_buf_length:	Number of bytes to send (length of 'tx_buf').
 * @param stop_flag:		Generate stop condition at the end of the transfer if non zero.
 * @return status:			Function execution status.
 */
I2C_status_t I2C1_write(unsigned char slave_address, unsigned char* tx_buf, unsigned char tx_buf_length, unsigned char stop_flag) {
	// Local variables.
	I2C_status_t status = I2C_SUCCESS;
	unsigned int loop_count = 0;
	unsigned char idx = 0;
	// Clear peripheral.
	I2C1_clear();
	// Wait for I2C bus to be ready.
	while (((I2C1 -> ISR) & (0b1 << 15)) != 0) {
		// Wait for BUSY='0' or timeout.
		loop_count++;
		if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
			status = I2C_ERROR_BUSY;
			goto errors;
		}
	}
	// Configure number of bytes to send.
	I2C1 -> CR2 &= 0xFF00FFFF; // Reset bits 16-23.
	I2C1 -> CR2 |= (tx_buf_length << 16); // NBYTES = tx_buf_length.
	// Send 7-bits slave address with write request.
	I2C1 -> CR2 &= ~(0b1 << 10); // Write request (RD_WRN='0').
	I2C1 -> CR2 &= 0xFFFFFC00; // Reset bits 0-9.
	I2C1 -> CR2 |= ((slave_address & 0x7F) << 1); // SADD = slave_address. Warning: the 7-bits address starts from bit 1!
	// Generate start condition.
	I2C1 -> CR2 |= (0b1 << 13); // START='1'.
	loop_count = 0;
	while (((I2C1 -> CR2) & (0b1 << 13)) != 0) {
		// Wait for START bit to be cleared by hardware or timeout.
		loop_count++;
		if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
			status = I2C_ERROR_START_BIT_CLEAR;
			goto errors;
		}
	}
	// Send bytes.
	loop_count = 0;
	while ((idx < tx_buf_length) && (((I2C1 -> ISR) & (0b1 << 4)) == 0)) {
		// Wait for transmit buffer to be empty (TXIS='1').
		if (((I2C1 -> ISR) & (0b1 << 1)) != 0) {
			// Send next byte.
			I2C1 -> TXDR = tx_buf[idx];
			idx++;
		}
		// Exit if timeout.
		loop_count++;
		if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
			status = I2C_ERROR_TX_BUFFER_EMPTY;
			goto errors;
		}
	}
	// Wait for last byte to be sent.
	loop_count = 0;
	while (((I2C1 -> ISR) & (0b1 << 6)) == 0) {
		// Wait for TC='1' or timeout.
		loop_count++;
		if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
			status = I2C_ERROR_TRANSFER_COMPLETE;
			goto errors;
		}
	}
	if (stop_flag != 0) {
		// Generate stop condition.
		I2C1 -> CR2 |= (0b1 << 14);
		loop_count = 0;
		while (((I2C1 -> ISR) & (0b1 << 5)) == 0) {
			// Wait for STOPF='1' or timeout.
			loop_count++;
			if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
				status = I2C_ERROR_STOP_DETECTION_FLAG;
				goto errors;
			}
		}
		// Clear flag.
		I2C1 -> ICR |= (0b1 << 5); // STOPCF='1'.
	}
errors:
	return status;
}

/* READ BYTES FROM I2C1 BUS (see algorithme on p.611 of RM0377 datasheet).
 * @param slave_address:	Slave address on 7 bits.
 * @param rx_buf:			Array that will contain the byte(s) to receive.
 * @param rx_buf_length:	Number of bytes to receive (length of 'rx_buf').
 * @return status:			Function execution status.
 */
I2C_status_t I2C1_read(unsigned char slave_address, unsigned char* rx_buf, unsigned char rx_buf_length) {
	// Local variables.
	I2C_status_t status = I2C_SUCCESS;
	unsigned int loop_count = 0;
	unsigned char idx = 0;
	// Clear peripheral.
	I2C1_clear();
	// Wait for I2C bus to be ready.
	while (((I2C1 -> ISR) & (0b1 << 15)) != 0) {
		// Wait for BUSY='0' or timeout.
		loop_count++;
		if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
			status = I2C_ERROR_BUSY;
			goto errors;
		}
	}
	// Configure number of bytes to send.
	I2C1 -> CR2 &= 0xFF00FFFF; // Reset bits 16-23.
	I2C1 -> CR2 |= (rx_buf_length << 16); // NBYTES = rx_buf_length.
	// Send 7-bits slave address with write request.
	I2C1 -> CR2 |= (0b1 << 10); // Read request (RD_WRN='1').
	I2C1 -> CR2 |= (0b1 << 12); // 7-bits mode.
	I2C1 -> CR2 &= 0xFFFFFC00; // Reset bits 0-9.
	I2C1 -> CR2 |= ((slave_address & 0x7F) << 1); // SADD = slave_address. Warning: the 7-bits address starts from bit 1!
	// Generate start condition.
	I2C1 -> CR2 |= (0b1 << 13); // START='1'.
	loop_count = 0;
	while (((I2C1 -> CR2) & (0b1 << 13)) != 0) {
		// Wait for START bit to be cleared by hardware or timeout.
		loop_count++;
		if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
			status = I2C_ERROR_START_BIT_CLEAR;
			goto errors;
		}
	}
	// Get bytes.
	loop_count = 0;
	while (idx < rx_buf_length) {
		// Wait for incoming data (RXNE='1').
		if (((I2C1 -> ISR) & (0b1 << 2)) != 0) {
			// Fill RX buffer with new byte */
			rx_buf[idx] = (I2C1 -> RXDR);
			idx++;
		}
		// Exit if timeout.
		loop_count++;
		if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
			status = I2C_ERROR_RX_TIMEOUT;
			goto errors;
		}
	}
	// Send a NACK and STOP condition after last byte.
	I2C1 -> CR2 |= (0b1 << 15);
	I2C1 -> CR2 |= (0b1 << 14);
	loop_count = 0;
	while (((I2C1 -> ISR) & (0b1 << 5)) == 0) {
		// Wait for STOPF='1' or timeout.
		loop_count++;
		if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
			status = I2C_ERROR_STOP_DETECTION_FLAG;
			goto errors;
		}
	}
	// Clear flag.
	I2C1 -> ICR |= (0b1 << 5); // STOPCF='1'.
errors:
	return status;
}
