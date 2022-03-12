/*
 * sht3x.c
 *
 *  Created on: 27 oct. 2018
 *      Author: Ludo
 */

#include "sht3x.h"

#include "i2c.h"
#include "lptim.h"

/*** SHT3x local macros ***/

#define SHT3X_FULL_SCALE	65535 // Data are 16-bits length (2^(16)-1).

/*** SHT3x local structures ***/

typedef struct {
	signed char temperature_degrees;
	unsigned char humidity_percent;
} SHT3X_context_t;

/*** SHT3x local global variables ***/

static SHT3X_context_t sht3x_ctx;

/*** SHT3x functions ***/

/* PERFORM TEMPERATURE AND HUMIDITY MEASUREMENTS.
 * @param:			None.
 * @return status:	Function execution status.
 */
SHT3X_status_t SHT3X_perform_measurements(unsigned char i2c_address) {
	// Local variables.
	SHT3X_status_t status = SHT3X_SUCCESS;
	I2C_status_t i2c1_status = I2C_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	unsigned char measure_command[2] = {0x24, 0x00};
	unsigned char measure_buf[6];
	unsigned int data_16bits = 0;
	// Reset results.
	sht3x_ctx.temperature_degrees = 0;
	sht3x_ctx.humidity_percent = 0;
	// Trigger high repeatability measurement with clock stretching disabled.
	i2c1_status = I2C1_write(i2c_address, measure_command, 2, 1);
	I2C1_status_check(SHT3X_ERROR_BASE_I2C);
	// Wait for conversion to complete (at least 15ms).
	lptim1_status = LPTIM1_delay_milliseconds(50, 0);
	LPTIM1_status_check(SHT3X_ERROR_BASE_LPTIM);
	// Read data.
	i2c1_status = I2C1_read(i2c_address, measure_buf, 6);
	I2C1_status_check(SHT3X_ERROR_BASE_I2C);
	// Compute temperature (TBC: verify checksum).
	data_16bits = (measure_buf[0] << 8) + measure_buf[1];
	sht3x_ctx.temperature_degrees = ((175 * data_16bits) / (SHT3X_FULL_SCALE)) - 45;
	// Compute humidity (TBC: verify checksum).
	data_16bits = (measure_buf[3] << 8) + measure_buf[4];
	sht3x_ctx.humidity_percent = (100 * data_16bits) / (SHT3X_FULL_SCALE);
errors:
	return status;
}

/* READ TEMPERATURE FROM SHT3X SENSOR.
 * @param temperature_degrees:	Pointer to signed byte that will contain temperature result (2-complement).
 * @return:						None.
 */
void SHT3X_get_temperature(signed char* temperature_degrees) {
	// Get result.
	(*temperature_degrees) = sht3x_ctx.temperature_degrees;
}

/* READ HUMIDTY FROM SHT3X SENSOR.
 * @param humidity_percent:		Pointer to byte that will contain humidity result (%).
 * @return:						None.
 */
void SHT3X_get_humidity(unsigned char* humidity_percent) {
	// Get result.
	(*humidity_percent) = sht3x_ctx.humidity_percent;
}
