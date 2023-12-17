/*
 * i2c.h
 *
 *  Created on: 12 may 2018
 *      Author: Ludo
 */

#ifndef __I2C_H__
#define __I2C_H__

#include "lptim.h"
#include "types.h"

/*** I2C structures ***/

/*!******************************************************************
 * \enum I2C_status_t
 * \brief I2C driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	I2C_SUCCESS = 0,
	I2C_ERROR_NULL_PARAMETER,
	I2C_ERROR_TIMEOUT,
	I2C_ERROR_BUSY,
	I2C_ERROR_START_BIT_CLEAR,
	I2C_ERROR_TX_BUFFER_EMPTY,
	I2C_ERROR_RX_TIMEOUT,
	I2C_ERROR_TRANSFER_COMPLETE,
	I2C_ERROR_STOP_DETECTION_FLAG,
	// Low level drivers errors.
	I2C_ERROR_BASE_LPTIM = 0x0100,
	// Last base value.
	I2C_ERROR_BASE_LAST = (I2C_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST)
} I2C_status_t;

/*** I2C functions ***/

/*!******************************************************************
 * \fn void I2C1_init(void)
 * \brief Init I2C1 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void I2C1_init(void);

/*!******************************************************************
 * \fn void I2C1_init(void)
 * \brief Release I2C1 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void I2C1_de_init(void);

/*!******************************************************************
 * \fn I2C_status_t I2C1_write(uint8_t slave_address, uint8_t* tx_buf, uint8_t tx_buf_length, uint8_t stop_flag)
 * \brief Write data on I2C1 bus.
 * \param[in]  	slave_address: 7-bits destination slave address.
 * \param[in]	data: Byte array to send.
 * \param[in]	data_size_bytes: Number of bytes to send.
 * \param[in]	stop_flag: Generate stop condition at the end of the transfer is non zero.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
I2C_status_t I2C1_write(uint8_t slave_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag);

/*!******************************************************************
 * \fn I2C_status_t I2C1_read(uint8_t slave_address, uint8_t* data, uint8_t data_size_bytes)
 * \brief Read data on I2C1 bus.
 * \param[in]  	slave_address: 7-bits destination slave address.
 * \param[in]	data: Byte array that will contain the read data.
 * \param[in]	data_size_bytes: Number of bytes to read.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
I2C_status_t I2C1_read(uint8_t slave_address, uint8_t* data, uint8_t data_size_bytes);

/*******************************************************************/
#define I2C1_exit_error(error_base) { if (i2c1_status != I2C_SUCCESS) { status = (error_base + i2c1_status); goto errors; } }

/*******************************************************************/
#define I2C1_stack_error(void) { if (i2c1_status != I2C_SUCCESS) { ERROR_stack_add(ERROR_BASE_I2C + i2c1_status); } }

/*******************************************************************/
#define I2C1_stack_exit_error(error_code) { if (i2c1_status != I2C_SUCCESS) { ERROR_stack_add(ERROR_BASE_I2C1 + i2c1_status); status = error_code; goto errors; } }

#endif /* __I2C_H__ */
