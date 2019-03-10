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
#ifdef HW1_0
void SPI1_SetClockPolarity(unsigned char polarity);
#endif
void SPI1_WriteByte(unsigned char tx_data);
void SPI1_ReadByte(unsigned char tx_data, unsigned char* rx_data);
#ifdef HW1_0
void SPI1_WriteShort(unsigned short tx_data);
void SPI1_ReadShort(unsigned short tx_data, unsigned short* rx_data);
#endif

#ifdef HW2_0
void SPI2_Init(void);
void SPI2_Enable(void);
void SPI2_Disable(void);
void SPI2_PowerOn(void);
void SPI2_PowerOff(void);
void SPI2_WriteShort(unsigned short tx_data);
void SPI2_ReadShort(unsigned short tx_data, unsigned short* rx_data);
#endif

#endif /* SPI_H_ */
