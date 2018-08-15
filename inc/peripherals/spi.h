/*
 * spi.h
 *
 *  Created on: 19 june 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_SPI_H_
#define PERIPHERALS_SPI_H_

/*** SPI structures ***/

typedef enum {
	SPI_DATA_8BITS,
	SPI_DATA_16BITS
} SPI_DataLength;

/*** SPI functions ***/

void SPI_Init(void);
void SPI_SetDataLength(SPI_DataLength spi_data_length);
void SPI_SendByte(unsigned char byte_to_send);
void SPI_ReadByte(unsigned char* byte_to_read);
void SPI_SendShort(unsigned short short_to_send);
void SPI_ReadShort(unsigned short* short_to_read);

#endif /* PERIPHERALS_SPI_H_ */
