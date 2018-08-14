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
	GPIOA -> MODER |= (0b10101010 << 8); // Configure PA4, PA5, PA6 and PA7 as alternate function.
	GPIOA -> AFRL &= 0x0000FFFF; // Link PA4, PA5, PA6 and PA7 to AF0.

	/* Configure peripheral */
	SPI1 -> CR1 = 0; // Disable peripheral before configuration (SPE='0').
	SPI1 -> CR1 |= (0b1 << 14); // Enable output (BIDIOE='1').
	SPI1 -> CR1 |= (0b1 << 2); // Master mode (MSTR='1').
	SPI1 -> CR2 = 0;
	SPI1 -> CR2 |= (0b1 << 2); // Enable output (SSOE='1').

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
