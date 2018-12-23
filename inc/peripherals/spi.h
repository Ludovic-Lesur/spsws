/*
 * spi.h
 *
 *  Created on: 19 june 2018
 *      Author: Ludo
 */

#ifndef SPI_H
#define SPI_H

/*** SPI functions ***/

void SPI1_Init(void);
void SPI1_Enable(void);
void SPI1_Disable(void);
void SPI1_PowerOn(void);
void SPI1_PowerOff(void);
void SPI1_SetClockPolarity(unsigned char polarity);
void SPI1_WriteByte(unsigned char tx_data);
void SPI1_ReadByte(unsigned char tx_data, unsigned char* rx_data);
void SPI1_WriteShort(unsigned short tx_data);
void SPI1_ReadShort(unsigned short tx_data, unsigned short* rx_data);

#endif /* SPI_H_ */
