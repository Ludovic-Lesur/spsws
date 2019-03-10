/*
 * sht3x.c
 *
 *  Created on: 27 oct. 2018
 *      Author: Ludo
 */

#include "sht3x.h"

#include "i2c.h"
#include "lptim.h"
#include "tim.h"

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

/*** SHT3x functions ***/

/* INIT SHT3X SENSOR.
 * @param:	None.
 * @return:	None.
 */
void SHT3X_Init(void) {

	/* Init context */
	sht3x_ctx.sht3x_temperature_degrees = 0;
	sht3x_ctx.sht3x_humidity_percent = 0;
}

/* PERFORM TEMPERATURE AND HUMIDITY MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void SHT3X_PerformMeasurements(void) {

	/* Trigger high repeatability measurement with clock streching disabled */
	I2C1_Enable();
	unsigned char measurement_command[2] = {0x24, 0x00};
	I2C1_Write(SHT3X_I2C_ADDRESS, measurement_command, 2);

	/* Read result */
	// Wait for conversion to complete (at least 15ms).
	LPTIM1_DelayMilliseconds(20);
	unsigned char measure_buf[6];
	I2C1_Read(SHT3X_I2C_ADDRESS, measure_buf, 6);
	I2C1_Disable();
	// Temperature (TBC: verify checksum).
	unsigned int temperature_16bits = (measure_buf[0] << 8) + measure_buf[1];
	sht3x_ctx.sht3x_temperature_degrees = ((175 * temperature_16bits) / (SHT3X_FULL_SCALE)) - 45;
	// Humidity (TBC: verify checksum).
	unsigned int humidity_16bits = (measure_buf[3] << 8) + measure_buf[4];
	sht3x_ctx.sht3x_humidity_percent = (100 * humidity_16bits) / (SHT3X_FULL_SCALE);
}

/* READ TEMPERATURE FROM SHT3X SENSOR.
 * @param temperature_degrees:	Pointer to byte that will contain temperature result (°C).
 * @return:						None.
 */
void SHT3X_GetTemperature(signed char* temperature_degrees) {

	/* Get result */
	(*temperature_degrees) = sht3x_ctx.sht3x_temperature_degrees;
}

/* READ HUMIDTY FROM SHT3X SENSOR.
 * @param humidity_percent:		Pointer to byte that will contain humidity result (%).
 * @return:						None.
 */
void SHT3X_GetHumidity(unsigned char* humidity_percent) {

	/* Get result */
	(*humidity_percent) = sht3x_ctx.sht3x_humidity_percent;
}
