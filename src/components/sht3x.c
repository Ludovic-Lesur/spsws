/*
 * sht3x.c
 *
 *  Created on: 27 oct. 2018
 *      Author: Ludo
 */

#include "sht3x.h"

#include "i2c.h"

/*** SHT3x local macros ***/

#define SHT3X_I2C_ADDRESS	0x44
#define SHT3X_FULL_SCALE	65535 // Data are 16-bits length (2^(16)-1).

/*** SHT3x local structures ***/

typedef struct {
	signed char sht3x_temperature_degrees;
	unsigned char sht3x_humidity_percent;
} SHT3X_Context;

/*** SHT3x local global variables ***/

static SHT3X_Context sht3x_ctx;

/*** SHT3x local functions ***/

/* PERFORM TEMPERATURE AND HUMIDITY MEASUREMENTS.
 * @param temperature_degrees:	Pointer to byte that will contain temperature result (°C).
 * @param humidity_percent:		Pointer to byte that will contain humidity result (%).
 * @return:						None.
 */
void SHT3X_PerformMeasurements(void) {

	/* Reset results */
	sht3x_ctx.sht3x_temperature_degrees = 0;
	sht3x_ctx.sht3x_humidity_percent = 0;

	/* Trigger high repeatability measurement with clock streching disabled */
	unsigned char measurement_command[2] = {0x24, 0x00};
	I2C_Write(SHT3X_I2C_ADDRESS, measurement_command, 2);

	/* Read result */
	unsigned char measure_buf[6];
	I2C_Read(SHT3X_I2C_ADDRESS, measure_buf, 6);
	// Temperature.
	unsigned int temperature_16bits = (measure_buf[0] << 8) + measure_buf[1];
	sht3x_ctx.sht3x_temperature_degrees = ((175*temperature_16bits) / (SHT3X_FULL_SCALE)) - 45;
	// Humidity.
	unsigned int humidity_16bits = (measure_buf[3] << 8) + measure_buf[4];
	sht3x_ctx.sht3x_humidity_percent = (100*humidity_16bits) / (SHT3X_FULL_SCALE);
}

/*** SHT3x functions ***/

/* READ TEMPERATURE FROM SHT3X SENSOR.
 * @param temperature_degrees:	Pointer to byte that will contain temperature result (°C).
 * @return:						None.
 */
void SHT3X_GetTemperature(signed char* temperature_degrees) {
	SHT3X_PerformMeasurements();
	(*temperature_degrees) = sht3x_ctx.sht3x_temperature_degrees;
}

/* READ HUMIDY FROM SHT3X SENSOR.
 * @param humidity_percent:		Pointer to byte that will contain humidity result (%).
 * @return:						None.
 */
void SHT3X_GetHumidity(unsigned char* humidity_percent) {
	SHT3X_PerformMeasurements();
	(*humidity_percent) = sht3x_ctx.sht3x_humidity_percent;
}
