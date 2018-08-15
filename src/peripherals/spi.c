/*
 * spi.c
 *
 *  Created on: 19 june 2018
 *      Author: Ludovic
 */

#include "spi.h"

#include "gpio_reg.h"
#include "rcc_reg.h"
#include "spi_reg.h"

/*** SPI functions ***/

/* CONFIGURE SPI1.
 * @param:	None.
 * @return:	None.
 */
void SPI_Init(void) {

	/* Enable peripheral clock */
	RCC -> APB2ENR |= (0b1 << 12); // SPI1EN='1'.

	/* Configure NSS, SCK, MISO and MOSI GPIOs */
	RCC -> IOPENR |= (0b1 << 0); // Enable GPIOA clock.
	GPIOA -> MODER &= 0xFFFF00FF; // Reset bits 8-15.
	GPIOA -> MODER |= (0b10101001 << 8); // Configure PA4 as output, PA5, PA6 and PA7 as alternate function.
	GPIOA -> AFRL &= 0x000FFFFF; // Link PA5, PA6 and PA7 to AF0.
	//GPIOA -> PUPDR &= 0xFFFF00FF; // Reset bits 8-15.
	//GPIOA -> PUPDR |= (0b01010101 << 8); // Pull-up resistors on all pins.
	GPIOA -> ODR |= (0b1 << 4); // Set NSS to high (idle state).

	/* Configure peripheral */
	SPI1 -> CR1 = 0; // Disable peripheral before configuration (SPE='0').
	SPI1 -> CR1 |= (0b1 << 2); // Master mode (MSTR='1').
	SPI1 -> CR1 |= (0b111 << 3); // Baud rate = PCLK2/256 = SYSCLK/256 = 62.5kHz.
	SPI1 -> CR1 &= ~(0b1 << 11); // 8-bits format (DFF='0') by default.
	SPI1 -> CR2 = 0;
	SPI1 -> CR2 |= (0b1 << 2); // Enable output (SSOE='1').

	/* Enable peripheral */
	SPI1 -> CR1 |= (0b1 << 6);
}

/* SET SPI TRANFER DATA LENGTH.
 * @param spi_data_length:	'SPI_DATA_8BITS' or 'SPI_DATA_16BITS'.
 * @return:					None.
 */
void SPI_SetDataLength(SPI_DataLength spi_data_length) {

	/* Disable peripheral */
	SPI1 -> CR1 &= ~(0b1 << 6);

	/* Configure data length */
	switch (spi_data_length) {
	case SPI_DATA_8BITS:
		// 8-bits format.
		SPI1 -> CR1 &= ~(0b1 << 11); // DFF='0'.
		break;
	case SPI_DATA_16BITS:
		// 16-bits format.
		SPI1 -> CR1 |= (0b1 << 11); // DFF='1'.
		break;
	default:
		// Unknown configuration.
		break;
	}

	/* Enable peripheral */
	SPI1 -> CR1 |= (0b1 << 6);
}

/* SEND A BYTE THROUGH SPI.
 * @param byte_to_send:	Byte to send.
 * @return:				None.
 */
void SPI_SendByte(unsigned char byte_to_send) {
	SPI1 -> DR = byte_to_send;
	while (((SPI1 -> SR) & (0b1 << 1)) == 0); // Wait for transmission to complete (TXE='1').
}

/* READ A BYTE FROM SPI.
 * @param byte_to_read:	Pointer to byte that will contain the byte to read.
 * @return:				None.
 */
void SPI_ReadByte(unsigned char* byte_to_read) {
	while (((SPI1 -> SR) & (0b1 << 0)) == 0); // Wait for incoming data (RXNE='1').
	(*byte_to_read) = SPI1 -> DR;
}

/* SEND A SHORT THROUGH SPI.
 * @param shprt_to_send:	Short to send.
 * @return:					None.
 */
void SPI_SendShort(unsigned short short_to_send) {
	SPI1 -> DR = short_to_send;
	while (((SPI1 -> SR) & (0b1 << 1)) == 0); // Wait for transmission to complete (TXE='1').
}

/* READ A SHORT FROM SPI.
 * @param short_to_read:	Pointer to short that will contain the short to read.
 * @return:					None.
 */
void SPI_ReadShort(unsigned short* short_to_read) {
	while (((SPI1 -> SR) & (0b1 << 0)) == 0); // Wait for incoming data (RXNE='1').
	(*short_to_read) = SPI1 -> DR;
}
