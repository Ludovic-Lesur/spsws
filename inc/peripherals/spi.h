/*
 * spi.h
 *
 *  Created on: 19 june 2018
 *      Author: Ludo
 */

#ifndef SPI_H
#define SPI_H

#include "lptim.h"

/*** SPI structures ***/

typedef enum {
	SPI_SUCCESS = 0,
	SPI_ERROR_TX_BUFFER_EMPTY,
	SPI_ERROR_RX_TIMEOUT,
	SPI_ERROR_BASE_LPTIM = 0x0100,
	SPI_ERROR_BASE_LAST = (SPI_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST)
} SPI_status_t;

/*** SPI functions ***/

void SPI1_init(void);
void SPI1_enable(void);
void SPI1_disable(void);
SPI_status_t SPI1_power_on(void);
SPI_status_t SPI1_power_off(void);
#ifdef HW1_0
void SPI1_set_clock_polarity(unsigned char polarity);
#endif
SPI_status_t SPI1_write_byte(unsigned char tx_data);
SPI_status_t SPI1_read_byte(unsigned char tx_data, unsigned char* rx_data);
#ifdef HW1_0
SPI_status_t SPI1_write_short(unsigned short tx_data);
SPI_status_t SPI1_read_short(unsigned short tx_data, unsigned short* rx_data);
#endif

#define SPI1_status_check(error_base) { if (spi_status != SPI_SUCCESS) { status = error_base + spi_status; goto errors; }}
#define SPI1_error_check() { ERROR_status_check(spi_status, SPI_SUCCESS, ERROR_BASE_SPI1); }

#ifdef HW2_0
void SPI2_init(void);
void SPI2_enable(void);
void SPI2_disable(void);
SPI_status_t SPI2_power_on(void);
SPI_status_t SPI2_power_off(void);
SPI_status_t SPI2_write_short(unsigned short tx_data);
SPI_status_t SPI2_read_short(unsigned short tx_data, unsigned short* rx_data);

#define SPI2_status_check(error_base) { if (spi_status != SPI_SUCCESS) { status = error_base + spi_status; goto errors; }}
#define SPI2_error_check() { ERROR_status_check(spi_status, SPI_SUCCESS, ERROR_BASE_SPI2); }
#endif



#endif /* SPI_H_ */
