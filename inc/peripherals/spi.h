/*
 * spi.h
 *
 *  Created on: 19 june 2018
 *      Author: Ludo
 */

#ifndef __SPI_H__
#define __SPI_H__

#include "lptim.h"
#include "types.h"

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
SPI_status_t SPI1_power_on(void);
void SPI1_power_off(void);
#ifdef HW1_0
void SPI1_set_clock_polarity(uint8_t polarity);
#endif
SPI_status_t SPI1_write_byte(uint8_t tx_data);
SPI_status_t SPI1_read_byte(uint8_t tx_data, uint8_t* rx_data);
#ifdef HW1_0
SPI_status_t SPI1_write_short(uint16_t tx_data);
SPI_status_t SPI1_read_short(uint16_t tx_data, uint16_t* rx_data);
#endif

#define SPI1_status_check(error_base) { if (spi_status != SPI_SUCCESS) { status = error_base + spi_status; goto errors; }}
#define SPI1_error_check() { ERROR_status_check(spi_status, SPI_SUCCESS, ERROR_BASE_SPI1); }
#define SPI1_error_check_print() { ERROR_status_check_print(spi_status, SPI_SUCCESS, ERROR_BASE_SPI1); }

#ifdef HW2_0
void SPI2_init(void);
SPI_status_t SPI2_power_on(void);
void SPI2_power_off(void);
SPI_status_t SPI2_write_short(uint16_t tx_data);
SPI_status_t SPI2_read_short(uint16_t tx_data, uint16_t* rx_data);

#define SPI2_status_check(error_base) { if (spi_status != SPI_SUCCESS) { status = error_base + spi_status; goto errors; }}
#define SPI2_error_check() { ERROR_status_check(spi_status, SPI_SUCCESS, ERROR_BASE_SPI2); }
#define SPI2_error_check_print() { ERROR_status_check_print(spi_status, SPI_SUCCESS, ERROR_BASE_SPI2); }
#endif



#endif /* __SPI_H__ */
