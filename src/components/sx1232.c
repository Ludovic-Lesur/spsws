/*
 * sx1232.c
 *
 *  Created on: 20 june 2018
 *      Author: Ludovic
 */

#include "sx1232.h"

#include "gpio_reg.h"
#include "rcc_reg.h"
#include "spi.h"
#include "sx1232_reg.h"

/*** SX1232 functions ***/

/* SX1232 SINGLE ACCESS WRITE FUNCTION.
 * @param addr:		Register address (7 bits).
 * @param valie:	Value to write in register.
 * @return:			None.
 */
void SX1232_WriteRegister(unsigned char addr, unsigned char value) {
	/* Check addr is a 7-bits value */
	if (addr < (0b1 << 7)) {
		/* Build SPI frame */
		unsigned char sx1232_spi_command = 0;
		sx1232_spi_command |= (0b1 << 7) | addr; // '1 A6 A5 A4 A3 A2 A1 A0' for a write access.
		/* Write access sequence */
		GPIOB -> ODR &= ~(0b1 << 0); // Falling edge on CS pin.
		SPI_WriteByte(sx1232_spi_command);
		SPI_WriteByte(value);
		GPIOB -> ODR |= (0b1 << 0); // Set CS pin.
	}
}

/* SX1232 SINGLE ACCESS READ FUNCTION.
 * @param addr:		Register address (7 bits).
 * @param value:	Pointer to byte that will contain the register Value to read.
 * @return:			None.
 */
void SX1232_ReadRegister(unsigned char addr, unsigned char* value) {
	/* Check addr is a 7-bits value */
	if (addr < (0b1 << 7)) {
		/* Build SPI frame */
		unsigned char sx1232_spi_command = 0;
		sx1232_spi_command |= addr; // '0 A6 A5 A4 A3 A2 A1 A0' for a read access.
		/* Write access sequence */
		GPIOB -> ODR &= ~(0b1 << 0); // Falling edge on CS pin.
		SPI_WriteByte(sx1232_spi_command);
		SPI_ReadByte(sx1232_spi_command, value);
		GPIOB -> ODR |= (0b1 << 0); // Set CS pin.
	}
}

/* INIT SX1232 TRANSCEIVER.
 * @param:	None.
 * @return:	None.
 */
void SX1232_Init(void) {

	/* Init SPI peripheral */
	SPI_Init();

	/* Configure CS GPIO */
	RCC -> IOPENR |= (0b11 << 1);
	GPIOB -> MODER &= ~(0b11 << 0); // Reset bits 0-1.
	GPIOB -> MODER |= (0b01 << 0);
	GPIOB -> ODR |= (0b1 << 0); // CS high (idle state).
}

void SX1232_Start(void) {

	/* Configure SPI */
	SPI_SetClockPolarity(0);

	/* Enable TCXO input */
	SX1232_WriteRegister(SX1232_REG_TCXO, 0x19);
	// TBC.
}
