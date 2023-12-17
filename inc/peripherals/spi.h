/*
 * spi.h
 *
 *  Created on: 19 jun. 2018
 *      Author: Ludo
 */

#ifndef __SPI_H__
#define __SPI_H__

#include "types.h"

/*** SPI structures ***/

/*!******************************************************************
 * \enum SPI_status_t
 * \brief SPI driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	SPI_SUCCESS = 0,
	SPI_ERROR_NULL_PARAMETER,
	SPI_ERROR_TX_BUFFER_EMPTY,
	SPI_ERROR_RX_TIMEOUT,
	// Last base value.
	SPI_ERROR_BASE_LAST = 0x0100
} SPI_status_t;

/*** SPI functions ***/

/*!******************************************************************
 * \fn void SPI1_init(void)
 * \brief Init SPI1 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void SPI1_init(void);

/*!******************************************************************
 * \fn void SPI1_de_init(void)
 * \brief Release SPI1 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void SPI1_de_init(void);

#ifdef HW1_0
/*!******************************************************************
 * \fn void SPI1_set_clock_polarity(uint8_t polarity)
 * \brief Configure SPI1 clock polarity.
 * \param[in]  	polarity: Polarity to apply.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void SPI1_set_clock_polarity(uint8_t polarity);
#endif

/*!******************************************************************
 * \fn SPI_status_t SPI1_write_read(uint16_t* tx_data, uint16_t* rx_data, uint8_t transfer_size)
 * \brief SPI1 16-bits data transfer function.
 * \param[in]	tx_data: Short array to send.
 * \param[in]	transfer_size: Number of shorts to send and receive.
 * \param[out] 	rx_data: Pointer to the received shorts.
 * \retval		Function execution status.
 *******************************************************************/
SPI_status_t SPI1_write_read(uint16_t* tx_data, uint16_t* rx_data, uint8_t transfer_size);

/*!******************************************************************
 * \fn void SPI1_write_16(uint16_t tx_data)
 * \brief Optimized SPI1 single short transfer function.
 * \param[in]	tx_data: Short to send.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void SPI1_write(uint16_t tx_data);

#ifdef HW2_0
/*!******************************************************************
 * \fn void SPI2_init(void)
 * \brief Init SPI2 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void SPI2_init(void);
#endif

#ifdef HW2_0
/*!******************************************************************
 * \fn void SPI2_de_init(void)
 * \brief Release SPI2 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void SPI2_de_init(void);
#endif

#ifdef HW2_0
/*!******************************************************************
 * \fn SPI_status_t SPI2_write_read(uint16_t* tx_data, uint16_t* rx_data, uint8_t transfer_size)
 * \brief SPI2 16-bits data transfer function.
 * \param[in]	tx_data: Short array to send.
 * \param[in]	transfer_size: Number of shorts to send and receive.
 * \param[out] 	rx_data: Pointer to the received shorts.
 * \retval		Function execution status.
 *******************************************************************/
SPI_status_t SPI2_write_read(uint16_t* tx_data, uint16_t* rx_data, uint8_t transfer_size);
#endif

/*******************************************************************/
#define SPI1_exit_error(error_base) { if (spi1_status != SPI_SUCCESS) { status = (error_base + spi1_status); goto errors; } }

/*******************************************************************/
#define SPI1_stack_error(void) { if (spi1_status != SPI_SUCCESS) { ERROR_stack_add(ERROR_BASE_SPI1 + spi1_status); } }

/*******************************************************************/
#define SPI1_stack_exit_error(error_code) { if (spi1_status != SPI_SUCCESS) { ERROR_stack_add(ERROR_BASE_SPI1 + spi1_status); status = error_code; goto errors; } }

/*******************************************************************/
#define SPI2_exit_error(error_base) { if (spi2_status != SPI_SUCCESS) { status = (error_base + spi2_status); goto errors; } }

/*******************************************************************/
#define SPI2_stack_error(void) { if (spi2_status != SPI_SUCCESS) { ERROR_stack_add(ERROR_BASE_SPI2 + spi2_status); } }

/*******************************************************************/
#define SPI2_stack_exit_error(error_code) { if (spi2_status != SPI_SUCCESS) { ERROR_stack_add(ERROR_BASE_SPI2 + spi2_status); status = error_code; goto errors; } }

#endif /* __SPI_H__ */
