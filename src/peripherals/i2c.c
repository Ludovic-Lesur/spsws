/*
 * i2c.c
 *
 *  Created on: 12 may 2018
 *      Author: Ludovic
 */

#include "i2c.h"

#include "gpio_reg.h"
#include "i2c_reg.h"
#include "rcc.h"
#include "rcc_reg.h"

/*** I2C local macros ***/

#define I2CCLK_KHZ	10

/*** I2C functions ***/

/* CONFIGURE I2C1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void I2C_Init(void) {

	/* Enable peripheral clock */
	RCC -> APB1ENR |= (0b1 << 21); // I2C1EN='1'.

	/* Configure SCL and SDA GPIOs */
	RCC -> IOPENR |= (0b1 << 0); // Enable GPIOA clock.
	GPIOA -> MODER &= ~(0b1111 << 18); // Reset bits 18-21.
	GPIOA -> MODER |= (0b1010 << 18); // Configure PA9 and PA10 as alternate function.
	GPIOA -> AFRH &= 0xFFFFF00F; // Reset bits 4-11.
	GPIOA -> AFRH |= 0x00000110; // Link PA9 and PA10 to AF1.

	/* Configure peripheral */
	I2C1 -> CR1 &= ~(0b1 << 0); // Disable peripheral before configuration (PE='0').
	I2C1 -> CR1 &= ~(0b11111 << 8); // Analog filter enabled (ANFOFF='0') and digital filter disabled (DNF='0000').
	I2C1 -> TIMINGR &= ~(0b1111 << 28); // I2CCLK = PCLK1 = SYSCLK (PRESC='0000').
	I2C1 -> TIMINGR &= 0xFF00FFFF; // No delay (SCLDEL='0000' and SDADEL='0000').
	I2C1 -> TIMINGR &= 0xFFFF0000; // Reset bits 0-15.
	unsigned int scl_half_period = ((SYSCLK_KHZ/I2CCLK_KHZ)-1)/2; // See p.641 of RM0377 datasheet.
	I2C1 -> TIMINGR |= (scl_half_period << 8) + scl_half_period; // Set SCL frequency.
	I2C1 -> CR1 &= ~(0b1 << 17); // Clock stretching enabled (NOSTRETCH='0').
	I2C1 -> CR2 &= ~(0b1 << 11); // 7-bits addressing mode (ADD10='0').

	/* Enable peripheral */
	I2C1 -> CR1 |= (0b1 << 0);
}

/* SEND BYTES TO A SLAVE OVER I2C BUS (see algorithme on p.607 of RM0377 datasheet).
 * @param slave_address:	Slave address on 7 bits.
 * @param tx_buf:			Array containing the byte(s) to send.
 * @param tx_buf_length:	Number of bytes to send (length of 'tx_buf').
 * @return:					None.
 */
void I2C_SendBytes(unsigned char slave_address, unsigned char* tx_buf, unsigned char tx_buf_length) {

	/* Configure number of bytes to send */
	I2C1 -> CR2 &= 0xFF00FFFF; // Reset bits 16-23.
	I2C1 -> CR2 |= (tx_buf_length << 16); // NBYTES = tx_buf_length.
	I2C1 -> CR2 |= (0b1 << 25); // Stop condition automatically generated when NBYTES is reached (AUTOEND='1').

	/* Send 7-bits slave address with write request */
	I2C1 -> CR2 &= ~(0b1 << 10); // Write request (RD_WRN='0').
	I2C1 -> CR2 &= 0xFFFFFC00; // Reset bits 0-9.
	I2C1 -> CR2 |= (slave_address & 0x7F); // SADD = slave_address.

	/* Generate start condition */
	I2C1 -> CR2 |= (0b1 << 13); // START='1'.

	/* Send bytes */
	unsigned char byte_idx = 0;
	while (byte_idx < tx_buf_length) {

		/* Wait for TX data register to be empty (TXE='1') */
		while (((I2C1 -> ISR) & (0b1 << 0)) == 0);

		/* Fill TX data register with new byte */
		I2C1 -> TXDR = tx_buf[byte_idx];
		byte_idx++;
	}

	/* Wait the end of last transmission (TC='1') */
	while (((I2C1 -> ISR) & (0b1 << 6)) == 0);
}

/* GET BYTES FROM A SLAVE OVER I2C BUS (see algorithme on p.611 of RM0377 datasheet).
 * @param slave_address:	Slave address on 7 bits.
 * @param rx_buf:			Array that will contain the byte(s) to receive.
 * @param rx_buf_length:	Number of bytes to receive (length of 'rx_buf').
 * @return:					None.
 */
void I2C_GetBytes(unsigned char slave_address, unsigned char* rx_buf, unsigned char rx_buf_length) {

	/* Configure number of bytes to send */
	I2C1 -> CR2 &= 0xFF00FFFF; // Reset bits 16-23.
	I2C1 -> CR2 |= (rx_buf_length << 16); // NBYTES = rx_buf_length.
	I2C1 -> CR2 |= (0b1 << 25); // Stop condition automatically generated when NBYTES is reached (AUTOEND='1').

	/* Send 7-bits slave address with write request */
	I2C1 -> CR2 |= (0b1 << 10); // Read request (RD_WRN='1').
	I2C1 -> CR2 &= 0xFFFFFC00; // Reset bits 0-9.
	I2C1 -> CR2 |= (slave_address & 0x7F); // SADD = slave_address.

	/* Generate start condition */
	I2C1 -> CR2 |= (0b1 << 13); // START='1'.

	/* Send bytes */
	unsigned char byte_idx = 0;
	while (byte_idx < rx_buf_length) {

		/* Wait for incoming data (RXNE='1') */
		while (((I2C1 -> ISR) & (0b1 << 2)) == 0);

		/* Fill RX buffer with new byte */
		rx_buf[byte_idx] = I2C1 -> RXDR;
		byte_idx++;
	}

	/* Wait the end of last transmission (TC='1') */
	while (((I2C1 -> ISR) & (0b1 << 6)) == 0);
}
