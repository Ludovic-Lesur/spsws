/*
 * i2c.h
 *
 *  Created on: 12 may 2018
 *      Author: Ludo
 */

#ifndef I2C_H
#define I2C_H

#include "gpio.h"
#include "lptim.h"

/*** I2C structures ***/

typedef enum {
	I2C_SUCCESS = 0,
	I2C_ERROR_LPTIM,
	I2C_ERROR_TIMEOUT = (I2C_ERROR_LPTIM + LPTIM_ERROR_LAST),
	I2C_ERROR_BUSY,
	I2C_ERROR_START_BIT_CLEAR,
	I2C_ERROR_TX_BUFFER_EMPTY,
	I2C_ERROR_RX_TIMEOUT,
	I2C_ERROR_TRANSFER_COMPLETE,
	I2C_ERROR_STOP_DETECTION_FLAG,
	I2C_ERROR_LAST
} I2C_status_t;

/*** I2C functions ***/

void I2C1_init(void);
void I2C1_disable(void);
I2C_status_t I2C1_power_on(void);
I2C_status_t I2C1_power_off(void);
I2C_status_t I2C1_write(unsigned char slave_address, unsigned char* tx_buf, unsigned char tx_buf_length, unsigned char stop_flag);
I2C_status_t I2C1_read(unsigned char slave_address, unsigned char* rx_buf, unsigned char rx_buf_length);

#define I2C_status_check(error_base) { if (i2c_status != I2C_SUCCESS) { status = error_base + i2c_status; goto errors; }}

#endif /* I2C_H */
