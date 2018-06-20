/*
 * spi.h
 *
 *  Created on: 19 june 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_SPI_H_
#define PERIPHERALS_SPI_H_

/*** SPI functions ***/

void SPI_Init(void);
void SPI_SendByte(unsigned char byte_to_send);
void SPI_ReadByte(unsigned char* byte_to_read);

#endif /* PERIPHERALS_SPI_H_ */
