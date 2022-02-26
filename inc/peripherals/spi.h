/*
 * spi.h
 *
 *  Created on: 19 june 2018
 *      Author: Ludo
 */

#ifndef SPI_H
#define SPI_H

/*** SPI functions ***/

void SPI1_init(void);
void SPI1_enable(void);
void SPI1_disable(void);
void SPI1_power_on(void);
void SPI1_power_off(void);
#ifdef HW1_0
void SPI1_set_clock_polarity(unsigned char polarity);
#endif
unsigned char SPI1_write_byte(unsigned char tx_data);
unsigned char SPI1_read_byte(unsigned char tx_data, unsigned char* rx_data);
#ifdef HW1_0
unsigned char SPI1_write_short(unsigned short tx_data);
unsigned char SPI1_read_short(unsigned short tx_data, unsigned short* rx_data);
#endif

#ifdef HW2_0
void SPI2_init(void);
void SPI2_enable(void);
void SPI2_disable(void);
void SPI2_power_on(void);
void SPI2_power_off(void);
unsigned char SPI2_write_short(unsigned short tx_data);
unsigned char SPI2_read_short(unsigned short tx_data, unsigned short* rx_data);
#endif

#endif /* SPI_H_ */
