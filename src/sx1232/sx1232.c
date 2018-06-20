/*
 * sx1232.c
 *
 *  Created on: 20 june 2018
 *      Author: Ludovic
 */

#include "gpio_reg.h"
#include "spi.h"
#include "sx1232.h"
#include "sx1232_reg.h"

/*** SX1232 local functions ***/

/* SX1232 SINGLE ACCESS WRITE FUNCTION.
 * @param addr:				Register address (7 bits).
 * @param data_to_write:	Value to write in register.
 * @return:					None.
 */
void SX1232_RegisterWrite(unsigned char addr, unsigned char data_to_write) {
	/* Check addr is a 7-bits value */
	if (addr < (0b1 << 7)) {
		/* Build SPI frame */
		unsigned char spi_command = (0b1 << 7) | addr; // '1 A6 A5 A4 A3 A2 A1 A0' for a write access.
		/* Write access sequence */
		GPIOA -> ODR &= ~(0b1 << 4); // Falling edge on NSS pin (OD4='0').
		SPI_SendByte(spi_command);
		SPI_SendByte(data_to_write);
		GPIOA -> ODR |= (0b1 << 4); // Set NSS pin (OD4='1').
	}
}

/* SX1232 SINGLE ACCESS READ FUNCTION.
 * @param addr:				Register address (7 bits).
 * @param data_to_read:		Pointer to byte that will contain the register Value to read.
 * @return:					None.
 */
void SX1232_RegisterRead(unsigned char addr, unsigned char* data_to_read) {
	/* Check addr is a 7-bits value */
	if (addr < (0b1 << 7)) {
		/* Build SPI frame */
		unsigned char spi_command = addr; // '0 A6 A5 A4 A3 A2 A1 A0' for a read access.
		/* Write access sequence */
		GPIOA -> ODR &= ~(0b1 << 4); // Falling edge on NSS pin (OD4='0').
		SPI_SendByte(spi_command);
		SPI_ReadByte(data_to_read);
		GPIOA -> ODR |= (0b1 << 4); // Set NSS pin (OD4='1').
	}
}

/*** SX1232 functions ***/

/* INIT SX1232 TRANSCEIVER.
 * @param:	None.
 * @return:	None.
 */
void SX1232_Init(void) {
	// TBC.
}
