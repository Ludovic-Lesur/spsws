/*
 * spi.h
 *
 *  Created on: 19 june 2018
 *      Author: Ludovic
 */

#ifndef SPI_H
#define SPI_H

/*** SPI functions ***/

void SPI_Init(void);
void SPI_PowerOn(void);
void SPI_PowerOff(void);
void SPI_SetClockPolarity(unsigned char polarity);
void SPI_WriteByte(unsigned char tx_data);
void SPI_ReadByte(unsigned char tx_data, unsigned char* rx_data);
void SPI_WriteShort(unsigned short tx_data);
void SPI_ReadShort(unsigned short tx_data, unsigned short* rx_data);

#endif /* PERIPHERALS_SPI_H_ */
