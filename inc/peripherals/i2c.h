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

typedef enum {
	I2C_SUCCESS = 0,
	I2C_ERROR_NULL_PARAMETER,
	I2C_ERROR_TIMEOUT,
	I2C_ERROR_BUSY,
	I2C_ERROR_START_BIT_CLEAR,
	I2C_ERROR_TX_BUFFER_EMPTY,
	I2C_ERROR_RX_TIMEOUT,
	I2C_ERROR_TRANSFER_COMPLETE,
	I2C_ERROR_STOP_DETECTION_FLAG,
	I2C_ERROR_LAST,
	I2C_ERROR_BASE_LPTIM = 0x0100,
	I2C_ERROR_BASE_LAST = (I2C_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST)
} I2C_status_t;

/*** I2C functions ***/

void I2C1_init(void);
I2C_status_t I2C1_power_on(void);
void I2C1_power_off(void);
I2C_status_t I2C1_write(uint8_t slave_address, uint8_t* tx_buf, uint8_t tx_buf_length, uint8_t stop_flag);
I2C_status_t I2C1_read(uint8_t slave_address, uint8_t* rx_buf, uint8_t rx_buf_length);

#define I2C1_status_check(error_base) { if (i2c1_status != I2C_SUCCESS) { status = error_base + i2c1_status; goto errors; }}
#define I2C1_error_check() { ERROR_status_check(i2c1_status, I2C_SUCCESS, ERROR_BASE_I2C1); }
#define I2C1_error_check_print() { ERROR_status_check_print(i2c1_status, I2C_SUCCESS, ERROR_BASE_I2C1); }

#endif /* __I2C_H__ */
