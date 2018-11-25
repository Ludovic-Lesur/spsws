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
#include "tim.h"

/*** SPI functions ***/

/* CONFIGURE SPI1.
 * @param:	None.
 * @return:	None.
 */
void SPI_Init(void) {

	/* Enable peripheral clock */
	RCC -> APB2ENR |= (0b1 << 12); // SPI1EN='1'.

	/* Configure SCK, MISO and MOSI GPIOs (NSS is configured in each component driver) */
	RCC -> IOPENR |= (0b1 << 0); // Enable GPIOA clock.
	GPIOA -> MODER &= 0xFFFF03FF; // Reset bits 10-15.
	GPIOA -> MODER |= (0b101010 << 10); // Configure PA5, PA6 and PA7 as alternate function.
	GPIOA -> AFRL &= 0x000FFFFF; // Link PA5, PA6 and PA7 to AF0.
	//GPIOA -> PUPDR &= 0xFFFF03FF; // Reset bits 10-15.
	//GPIOA -> PUPDR |= (0b01 << 12); // Pull-up resistors on all pins.

	/* Configure peripheral */
	SPI1 -> CR1 = 0; // Disable peripheral before configuration (SPE='0').
	SPI1 -> CR1 |= (0b1 << 2); // Master mode (MSTR='1').
	SPI1 -> CR1 |= (0b011 << 3); // Baud rate = PCLK2/16 = SYSCLK/16 = 1MHz.
	SPI1 -> CR1 &= ~(0b1 << 11); // 8-bits format (DFF='0') by default.
	//SPI1 -> CR1 |= (0b1 << 9); // Software slave management enabled (SSM='1').
	SPI1 -> CR2 = 0;
	SPI1 -> CR2 |= (0b1 << 2); // Enable output (SSOE='1').

	/* Enable peripheral */
	SPI1 -> CR1 |= (0b1 << 6);
}

/* SET SPI SCLK POLARITY.
 * @param polarity:	Clock polarity (0 = SCLK idle high, otherwise SCLK idle low).
 * @return:			None.
 */
void SPI_SetClockPolarity(unsigned char polarity) {
	if (polarity == 0) {
		SPI1 -> CR1 &= ~(0b11 << 0); // CPOL='0' and CPHA='0'.
	}
	else {
		SPI1 -> CR1 |= (0b11 << 0); // CPOL='1' and CPHA='1'.
	}
}

/* SWITCH ALL SPI SLAVES ON.
 * @param:	None.
 * @return:	None.
 */
void SPI_PowerOn(void) {

	/* Enable RF power supply */
	GPIOB -> MODER &= ~(0b11 << 2); // Reset bits 2-3.
	GPIOB -> MODER |= (0b01 << 2);
	GPIOB -> ODR |= (0b1 << 1);
	TIM22_WaitMilliseconds(100);

	/* Enable sensors power supply */
	GPIOB -> MODER &= ~(0b11 << 10); // Reset bits 8-9.
	GPIOB -> MODER |= (0b01 << 10);
	GPIOB -> ODR |= (0b1 << 5);
	TIM22_WaitMilliseconds(100);
}

/* SEND A BYTE THROUGH SPI.
 * @param tx_data:	Data to send (8-bits).
 * @return:			None.
 */
void SPI_WriteByte(unsigned char tx_data) {
	/* Set data length to 8-bits */
	SPI1 -> CR1 &= ~(0b1 << 11); // DFF='0'.

	/* Send data */
	*((volatile unsigned char*) &(SPI1 -> DR)) = tx_data;

	/* Wait for transmission to complete */
	while ((((SPI1 -> SR) & (0b1 << 1)) == 0) || (((SPI1 -> SR) & (0b1 << 7)) != 0)); // Wait for TXE='1' and BSY='0'.
}

/* SEND A SHORT THROUGH SPI.
 * @param tx_data:	Data to send (16-bits).
 * @return:			None.
 */
void SPI_WriteShort(unsigned short tx_data) {
	/* Set data length to 16-bits */
	SPI1 -> CR1 |= (0b1 << 11); // DFF='1'.

	/* Send data */
	*((volatile unsigned short*) &(SPI1 -> DR)) = tx_data;

	/* Wait for transmission to complete */
	while ((((SPI1 -> SR) & (0b1 << 1)) == 0) || (((SPI1 -> SR) & (0b1 << 7)) != 0)); // Wait for TXE='1' and BSY='0'.
}

/* READ A BYTE FROM SPI.
 * @param rx_data:	Pointer to byte that will contain the data to read (8-bits).
 * @return:			None.
 */
void SPI_ReadByte(unsigned char tx_data, unsigned char* rx_data) {
	/* Set data length to 8-bits */
	SPI1 -> CR1 &= ~(0b1 << 11); // DFF='0'.

	/* Send dummy data on MOSI to generate clock */
	*((volatile unsigned char*) &(SPI1 -> DR)) = tx_data;

	/* Wait for incoming data */
	while (((SPI1 -> SR) & (0b1 << 0)) == 0); // Wait for RXNE='1'.

	/* Read data */
	(*rx_data) = *((volatile unsigned char*) &(SPI1 -> DR));

	/* Wait for transfers to complete */
	while ((((SPI1 -> SR) & (0b1 << 1)) == 0) || (((SPI1 -> SR) & (0b1 << 7)) != 0)); // Wait for TXE='1' and BSY='0'.
}

/* READ A SHORT FROM SPI.
 * @param rx_data:	Pointer to short that will contain the data to read (16-bits).
 * @return:			None.
 */
void SPI_ReadShort(unsigned short tx_data, unsigned short* rx_data) {
	/* Set data length to 16-bits */
	SPI1 -> CR1 |= (0b1 << 11); // DFF='1'.

	/* Send dummy data on MOSI to generate clock */
	*((volatile unsigned short*) &(SPI1 -> DR)) = tx_data;

	/* Wait for incoming data */
	while (((SPI1 -> SR) & (0b1 << 0)) == 0); // Wait for RXNE='1'.

	/* Read data */
	(*rx_data) = *((volatile unsigned short*) &(SPI1 -> DR));

	/* Wait for transfers to complete */
	while ((((SPI1 -> SR) & (0b1 << 1)) == 0) || (((SPI1 -> SR) & (0b1 << 7)) != 0)); // Wait for TXE='1' and BSY='0'.
}
