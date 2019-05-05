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
#include "tim.h"

/*** I2C local macros ***/

#define I2C_ACCESS_TIMEOUT_SECONDS	3

/*** I2C functions ***/

/* CONFIGURE I2C1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void I2C1_Init(void) {

	/* Enable peripheral clock */
	RCC -> APB1ENR |= (0b1 << 21); // I2C1EN='1'.

	/* Configure power enable pin */
	GPIO_Configure(&GPIO_SENSORS_POWER_ENABLE, Output, PushPull, LowSpeed, NoPullUpNoPullDown);
	GPIO_Write(&GPIO_SENSORS_POWER_ENABLE, 0);

	/* Configure SCL and SDA (first as high impedance) */
	GPIO_Configure(&GPIO_I2C1_SCL, Analog, OpenDrain, LowSpeed, NoPullUpNoPullDown);
	GPIO_Configure(&GPIO_I2C1_SDA, Analog, OpenDrain, LowSpeed, NoPullUpNoPullDown);

	/* Configure peripheral */
	I2C1 -> CR1 &= ~(0b1 << 0); // Disable peripheral before configuration (PE='0').
	I2C1 -> CR1 &= ~(0b11111 << 8); // Analog filter enabled (ANFOFF='0') and digital filter disabled (DNF='0000').
	I2C1 -> TIMINGR = 0; // Reset all bits.
	I2C1 -> TIMINGR |= (7 << 28); // I2CCLK = PCLK1/(PRESC+1) = SYSCLK/(PRESC+1) = 16/(7+1) = 2MHz. (PRESC='1000').
	I2C1 -> TIMINGR |= (99 << 8) + 99; // Set SCL frequency to 10kHz (see p.641 of RM0377 datasheet).
	I2C1 -> CR1 &= ~(0b1 << 17); // Must be kept cleared in master mode (NOSTRETCH='0').
	I2C1 -> CR2 &= ~(0b1 << 11); // 7-bits addressing mode (ADD10='0').

	/* Enable peripheral */
	I2C1 -> CR1 |= (0b1 << 0); // PE='1'.
}

/* DISABLE I2C PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void I2C1_Disable(void) {

	/* Disable I2C1 peripheral */
	I2C1 -> CR1 &= ~(0b1 << 0);
	RCC -> APB1ENR &= ~(0b1 << 21); // I2C1EN='0'.
}

/* SWITCH ALL I2C1 SLAVES ON.
 * @param:	None.
 * @return:	None.
 */
void I2C1_PowerOn(void) {

	/* Enable GPIOs */
	GPIO_Configure(&GPIO_I2C1_SCL, AlternateFunction, OpenDrain, LowSpeed, NoPullUpNoPullDown);
	GPIO_Configure(&GPIO_I2C1_SDA, AlternateFunction, OpenDrain, LowSpeed, NoPullUpNoPullDown);

	/* Switch SHT3x, DPS310 and SI1133 on */
	GPIO_Write(&GPIO_SENSORS_POWER_ENABLE, 1);
	LPTIM1_DelayMilliseconds(100);
}

/* SWITCH ALL I2C1 SLAVES ON.
 * @param:	None.
 * @return:	None.
 */
void I2C1_PowerOff(void) {

	/* Switch SHT3x, DPS310 and SI1133 off */
	GPIO_Write(&GPIO_SENSORS_POWER_ENABLE, 0);

	/* Disable I2C alternate function */
	GPIO_Configure(&GPIO_I2C1_SCL, Input, PushPull, LowSpeed, NoPullUpNoPullDown);
	GPIO_Configure(&GPIO_I2C1_SDA, Input, PushPull, LowSpeed, NoPullUpNoPullDown);

	/* Delay required if another cycle is requested by applicative layer */
	LPTIM1_DelayMilliseconds(100);
}

/* WRITE DATA ON I2C1 BUS (see algorithme on p.607 of RM0377 datasheet).
 * @param slave_address:	Slave address on 7 bits.
 * @param tx_buf:			Array containing the byte(s) to send.
 * @param tx_buf_length:	Number of bytes to send (length of 'tx_buf').
 * @return:					1 in case of success, 0 in case of failure.
 */
unsigned char I2C1_Write(unsigned char slave_address, unsigned char* tx_buf, unsigned char tx_buf_length) {

	/* Wait for I2C bus to be ready */
	unsigned int loop_start_time = TIM22_GetSeconds();
	while (((I2C1 -> ISR) & (0b1 << 15)) != 0) {
		// Wait for BUSY='0' or timeout.
		if (TIM22_GetSeconds() > (loop_start_time + I2C_ACCESS_TIMEOUT_SECONDS)) return 0;
	}

	/* Configure number of bytes to send */
	I2C1 -> CR2 &= 0xFF00FFFF; // Reset bits 16-23.
	I2C1 -> CR2 |= (tx_buf_length << 16); // NBYTES = tx_buf_length.

	/* Send 7-bits slave address with write request */
	I2C1 -> CR2 &= ~(0b1 << 10); // Write request (RD_WRN='0').
	I2C1 -> CR2 &= 0xFFFFFC00; // Reset bits 0-9.
	I2C1 -> CR2 |= ((slave_address & 0x7F) << 1); // SADD = slave_address. Warning: the 7-bits address starts from bit 1!

	/* Generate start condition */
	I2C1 -> CR2 |= (0b1 << 13); // START='1'.
	loop_start_time = TIM22_GetSeconds();
	while (((I2C1 -> CR2) & (0b1 << 13)) != 0) {
		// Wait for START bit to be cleared by hardware or timeout.
		if (TIM22_GetSeconds() > (loop_start_time + I2C_ACCESS_TIMEOUT_SECONDS)) return 0;
	}

	/* Send bytes */
	unsigned char byte_idx = 0;
	loop_start_time = TIM22_GetSeconds();
	while ((byte_idx < tx_buf_length) && (((I2C1 -> ISR) & (0b1 << 4)) == 0)) {
		// Wait for transmit buffer to be empty (TXIS='1').
		if (((I2C1 -> ISR) & (0b1 << 1)) != 0) {
			// Send next byte.
			I2C1 -> TXDR = tx_buf[byte_idx];
			byte_idx++;
		}
		// Exit if timeout.
		if (TIM22_GetSeconds() > (loop_start_time + I2C_ACCESS_TIMEOUT_SECONDS)) return 0;
	}

	/* Wait for last byte to be sent */
	loop_start_time = TIM22_GetSeconds();
	while (((I2C1 -> ISR) & (0b1 << 6)) == 0) {
		// Wait for TC='1' or timeout.
		if (TIM22_GetSeconds() > (loop_start_time + I2C_ACCESS_TIMEOUT_SECONDS)) return 0;
	}

	/* Generate stop condition. */
	I2C1 -> CR2 |= (0b1 << 14);
	loop_start_time = TIM22_GetSeconds();
	while (((I2C1 -> ISR) & (0b1 << 5)) == 0) {
		// Wait for STOPF='1' or timeout.
		if (TIM22_GetSeconds() > (loop_start_time + I2C_ACCESS_TIMEOUT_SECONDS)) return 0;
	}
	// Clear flag.
	I2C1 -> ICR |= (0b1 << 5); // STOPCF='1'.
	return 1;
}

/* READ BYTES FROM I2C1 BUS (see algorithme on p.611 of RM0377 datasheet).
 * @param slave_address:	Slave address on 7 bits.
 * @param rx_buf:			Array that will contain the byte(s) to receive.
 * @param rx_buf_length:	Number of bytes to receive (length of 'rx_buf').
 * @return:					1 in case of success, 0 in case of failure.
 */
unsigned char I2C1_Read(unsigned char slave_address, unsigned char* rx_buf, unsigned char rx_buf_length) {

	/* Wait for I2C bus to be ready */
	unsigned int loop_start_time = TIM22_GetSeconds();
	while (((I2C1 -> ISR) & (0b1 << 15)) != 0) {
		// Wait for BUSY='0' or timeout.
		if (TIM22_GetSeconds() > (loop_start_time + I2C_ACCESS_TIMEOUT_SECONDS)) return 0;
	}

	/* Configure number of bytes to send */
	I2C1 -> CR2 &= 0xFF00FFFF; // Reset bits 16-23.
	I2C1 -> CR2 |= (rx_buf_length << 16); // NBYTES = rx_buf_length.

	/* Send 7-bits slave address with write request */
	I2C1 -> CR2 |= (0b1 << 10); // Read request (RD_WRN='1').
	I2C1 -> CR2 |= (0b1 << 12); // 7-bits mode.
	I2C1 -> CR2 &= 0xFFFFFC00; // Reset bits 0-9.
	I2C1 -> CR2 |= ((slave_address & 0x7F) << 1); // SADD = slave_address. Warning: the 7-bits address starts from bit 1!

	/* Generate start condition */
	I2C1 -> CR2 |= (0b1 << 13); // START='1'.
	loop_start_time = TIM22_GetSeconds();
	while (((I2C1 -> CR2) & (0b1 << 13)) != 0) {
		// Wait for START bit to be cleared by hardware or timeout.
		if (TIM22_GetSeconds() > (loop_start_time + I2C_ACCESS_TIMEOUT_SECONDS)) return 0;
	}

	/* Get bytes */
	unsigned char byte_idx = 0;
	loop_start_time = TIM22_GetSeconds();
	while (byte_idx < rx_buf_length) {
		// Wait for incoming data (RXNE='1').
		if (((I2C1 -> ISR) & (0b1 << 2)) != 0) {
			// Fill RX buffer with new byte */
			rx_buf[byte_idx] = I2C1 -> RXDR;
			byte_idx++;
		}
		// Exit if timeout.
		if (TIM22_GetSeconds() > (loop_start_time + I2C_ACCESS_TIMEOUT_SECONDS)) return 0;
	}

	/* Send a NACK and STOP condition after last byte */
	I2C1 -> CR2 |= (0b1 << 15);
	I2C1 -> CR2 |= (0b1 << 14);
	loop_start_time = TIM22_GetSeconds();
	while (((I2C1 -> ISR) & (0b1 << 5)) == 0) {
		// Wait for STOPF='1' or timeout.
		if (TIM22_GetSeconds() > (loop_start_time + I2C_ACCESS_TIMEOUT_SECONDS)) return 0;
	}
	// Clear flag.
	I2C1 -> ICR |= (0b1 << 5); // STOPCF='1'.
	return 1;
}
