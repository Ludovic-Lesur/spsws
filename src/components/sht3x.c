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
	unsigned char temperature_degrees_comp1;
	signed char temperature_degrees_comp2;
	unsigned char humidity_percent;
} SHT3X_context_t;

/*** SHT3x local global variables ***/

static SHT3X_context_t sht3x_ctx;

/*** SHT3x functions ***/

/* PERFORM TEMPERATURE AND HUMIDITY MEASUREMENTS.
 * @param:			None.
 * @return status:	Function execution status.
 */
SHTX_status_t SHT3X_perform_measurements(unsigned char i2c_address) {
	// Local variables.
	SHTX_status_t status = SHT3X_SUCCESS;
	I2C_status_t i2c1_status = I2C_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	unsigned char measure_command[2] = {0x24, 0x00};
	unsigned char measure_buf[6];
	unsigned int data_16bits = 0;
	unsigned char temperature_abs = 0;
	// Reset results.
	sht3x_ctx.temperature_degrees_comp2 = 0;
	sht3x_ctx.temperature_degrees_comp1 = 0;
	sht3x_ctx.humidity_percent = 0;
	// Trigger high repeatability measurement with clock stretching disabled.
	i2c1_status = I2C1_write(i2c_address, measure_command, 2, 1);
	I2C1_status_check(SHT3X_ERROR_I2C);
	// Wait for conversion to complete (at least 15ms).
	lptim1_status = LPTIM1_delay_milliseconds(50, 0);
	LPTIM1_status_check(SHT3X_ERROR_LPTIM);
	// Read data.
	i2c1_status = I2C1_read(i2c_address, measure_buf, 6);
	I2C1_status_check(SHT3X_ERROR_I2C);
	// Compute temperature (TBC: verify checksum).
	data_16bits = (measure_buf[0] << 8) + measure_buf[1];
	sht3x_ctx.temperature_degrees_comp2 = ((175 * data_16bits) / (SHT3X_FULL_SCALE)) - 45;
	// Convert to 1-complement value.
	sht3x_ctx.temperature_degrees_comp1 = 0;
	if (sht3x_ctx.temperature_degrees_comp2 < 0) {
		sht3x_ctx.temperature_degrees_comp1 |= 0x80;
		temperature_abs = (-1) * (sht3x_ctx.temperature_degrees_comp2);
		sht3x_ctx.temperature_degrees_comp1 |= (temperature_abs & 0x7F);
	}
	else {
		sht3x_ctx.temperature_degrees_comp1 = (sht3x_ctx.temperature_degrees_comp2 & 0x7F);
	}
	// Compute humidity (TBC: verify checksum).
	data_16bits = (measure_buf[3] << 8) + measure_buf[4];
	sht3x_ctx.humidity_percent = (100 * data_16bits) / (SHT3X_FULL_SCALE);
errors:
	return status;
}

/* READ TEMPERATURE FROM SHT3X SENSOR.
 * @param temperature_degrees:	Pointer to byte that will contain temperature result (1-complement).
 * @return:						None.
 */
void SHT3X_get_temperature_comp1(unsigned char* temperature_degrees) {
	// Get result.
	(*temperature_degrees) = sht3x_ctx.temperature_degrees_comp1;
}

/* READ TEMPERATURE FROM SHT3X SENSOR.
 * @param temperature_degrees:	Pointer to signed byte that will contain temperature result (2-complement).
 * @return:						None.
 */
void SHT3X_get_temperature_comp2(signed char* temperature_degrees) {
	// Get result.
	(*temperature_degrees) = sht3x_ctx.temperature_degrees_comp2;
}

/* READ HUMIDTY FROM SHT3X SENSOR.
 * @param humidity_percent:		Pointer to byte that will contain humidity result (%).
 * @return:						None.
 */
void SHT3X_get_humidity(unsigned char* humidity_percent) {
	// Get result.
	(*humidity_percent) = sht3x_ctx.humidity_percent;
}
