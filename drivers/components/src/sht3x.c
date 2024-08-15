/*
 * sht3x.c
 *
 *  Created on: 27 oct. 2018
 *      Author: Ludo
 */

#include "sht3x.h"

#include "gpio_mapping.h"
#include "i2c.h"
#include "lptim.h"
#include "types.h"

/*** SHT3x local macros ***/

#define SHT3X_I2C_INSTANCE	I2C_INSTANCE_I2C1

#define SHT3X_FULL_SCALE	65535 // Data are 16-bits.

/*** SHT3x local structures ***/

/*******************************************************************/
typedef struct {
	int32_t temperature_degrees;
	int32_t humidity_percent;
} SHT3X_context_t;

/*** SHT3x local global variables ***/

static SHT3X_context_t sht3x_ctx;

/*** SHT3x functions ***/

/*******************************************************************/
SHT3X_status_t SHT3X_init(void) {
	// Local variables.
	SHT3X_status_t status = SHT3X_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
	// Init I2C.
	i2c_status = I2C_init(SHT3X_I2C_INSTANCE, &GPIO_SENSORS_I2C);
	I2C_exit_error(SHT3X_ERROR_BASE_I2C);
errors:
	return status;
}

/*******************************************************************/
SHT3X_status_t SHT3X_de_init(void) {
	// Local variables.
	SHT3X_status_t status = SHT3X_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
	// Init I2C.
	i2c_status = I2C_de_init(SHT3X_I2C_INSTANCE, &GPIO_SENSORS_I2C);
	I2C_exit_error(SHT3X_ERROR_BASE_I2C);
errors:
	return status;
}

/*******************************************************************/
SHT3X_status_t SHT3X_perform_measurements(uint8_t i2c_address) {
	// Local variables.
	SHT3X_status_t status = SHT3X_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
	LPTIM_status_t lptim_status = LPTIM_SUCCESS;
	uint8_t measure_command[2] = {0x24, 0x00};
	uint8_t measure_buf[6];
	int32_t data_16bits = 0;
	// Reset results.
	sht3x_ctx.temperature_degrees = 0;
	sht3x_ctx.humidity_percent = 0;
	// Trigger high repeatability measurement with clock stretching disabled.
	i2c_status = I2C_write(SHT3X_I2C_INSTANCE, i2c_address, measure_command, 2, 1);
	I2C_exit_error(SHT3X_ERROR_BASE_I2C);
	// Wait for conversion to complete (at least 15ms).
	lptim_status = LPTIM_delay_milliseconds(30, LPTIM_DELAY_MODE_STOP);
	LPTIM_exit_error(SHT3X_ERROR_BASE_LPTIM);
	// Read data.
	i2c_status = I2C_read(SHT3X_I2C_INSTANCE, i2c_address, measure_buf, 6);
	I2C_exit_error(SHT3X_ERROR_BASE_I2C);
	// Compute temperature.
	data_16bits = (int32_t) ((measure_buf[0] << 8) + measure_buf[1]);
	sht3x_ctx.temperature_degrees = ((175 * data_16bits) / (SHT3X_FULL_SCALE)) - 45;
	// Compute humidity.
	data_16bits = (int32_t) ((measure_buf[3] << 8) + measure_buf[4]);
	sht3x_ctx.humidity_percent = (100 * data_16bits) / (SHT3X_FULL_SCALE);
errors:
	return status;
}

/*******************************************************************/
SHT3X_status_t SHT3X_get_temperature(int32_t* temperature_degrees) {
	// Local variables.
	SHT3X_status_t status = SHT3X_SUCCESS;
	// Check parameter.
	if (temperature_degrees == NULL) {
		status = SHT3X_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Get result.
	(*temperature_degrees) = sht3x_ctx.temperature_degrees;
errors:
	return status;
}

/*******************************************************************/
SHT3X_status_t SHT3X_get_humidity(int32_t* humidity_percent) {
	// Local variables.
	SHT3X_status_t status = SHT3X_SUCCESS;
	// Check parameter.
	if (humidity_percent == NULL) {
		status = SHT3X_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Get result.
	(*humidity_percent) = sht3x_ctx.humidity_percent;
errors:
	return status;
}
