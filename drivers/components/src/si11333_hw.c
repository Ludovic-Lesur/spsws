/*
 * si1133_hw.c
 *
 *  Created on: 31 aug. 2024
 *      Author: Ludo
 */

#include "si1133_hw.h"

#include "error.h"
#include "gpio_mapping.h"
#include "i2c.h"
#include "lptim.h"
#include "si1133.h"
#include "types.h"

/*** SI1133 HW local macros ***/

#define SI1133_I2C_INSTANCE		I2C_INSTANCE_I2C1

/*** SI1133 HW functions ***/

/*******************************************************************/
SI1133_status_t SI1133_HW_init(void) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
	// Init I2C.
	i2c_status = I2C_init(SI1133_I2C_INSTANCE, &GPIO_SENSORS_I2C);
	I2C_exit_error(SI1133_ERROR_BASE_I2C);
errors:
	return status;
}

/*******************************************************************/
SI1133_status_t SI1133_HW_de_init(void) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
	// Init I2C.
	i2c_status = I2C_de_init(SI1133_I2C_INSTANCE, &GPIO_SENSORS_I2C);
	I2C_exit_error(SI1133_ERROR_BASE_I2C);
errors:
	return status;
}

/*******************************************************************/
SI1133_status_t SI1133_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
	// I2C transfer.
	i2c_status = I2C_write(SI1133_I2C_INSTANCE, i2c_address, data, data_size_bytes, stop_flag);
	I2C_exit_error(SI1133_ERROR_BASE_I2C);
errors:
	return status;
}

/*******************************************************************/
SI1133_status_t SI1133_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
	// I2C transfer.
	i2c_status = I2C_read(SI1133_I2C_INSTANCE, i2c_address, data, data_size_bytes);
	I2C_exit_error(SI1133_ERROR_BASE_I2C);
errors:
	return status;
}

/*******************************************************************/
SI1133_status_t SI1133_HW_delay_milliseconds(uint32_t delay_ms) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	LPTIM_status_t lptim_status = LPTIM_SUCCESS;
	// Perform delay.
	lptim_status = LPTIM_delay_milliseconds(delay_ms, LPTIM_DELAY_MODE_SLEEP);
	LPTIM_exit_error(SI1133_ERROR_BASE_DELAY);
errors:
	return status;
}
