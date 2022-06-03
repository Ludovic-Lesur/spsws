/*
 * dps310.h
 *
 *  Created on: 27 nov. 2018
 *      Author: Ludo
 */

#ifndef DPS310_H
#define DPS310_H

#include "i2c.h"
#include "lptim.h"
#include "math.h"

/*** DPS310 macros ***/

#define DPS310_EXTERNAL_I2C_ADDRESS		0x77

/*** DPS310 structures ***/

typedef enum {
	DPS310_SUCCESS = 0,
	DPS310_ERROR_COEFFICIENTS_TIMEOUT,
	DPS310_ERROR_SENSOR_TIMEOUT,
	DPS310_ERROR_TEMPERATURE_TIMEOUT,
	DPS310_ERROR_PRESSURE_TIMEOUT,
	DPS310_ERROR_BASE_I2C = 0x0100,
	DPS310_ERROR_BASE_LPTIM = (DPS310_ERROR_BASE_I2C + I2C_ERROR_BASE_LAST),
	DPS310_ERROR_BASE_MATH = (DPS310_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
	DPS310_ERROR_BASE_LAST = (DPS310_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST)
} DPS310_status_t;

/*** DPS310 functions ***/

void DPS310_init(void);
DPS310_status_t DPS310_perform_measurements(unsigned char i2c_address);
void DPS310_get_pressure(unsigned int* pressure_pa);
void DPS310_get_temperature(signed char* temperature_degrees);

#define DPS310_status_check(error_base) { if (dps310_status != DPS310_SUCCESS) { status = error_base + dps310_status; goto errors; }}
#define DPS310_error_check() { ERROR_status_check(dps310_status, DPS310_SUCCESS, ERROR_BASE_DPS310); }
#define DPS310_error_check_print() { ERROR_status_check_print(dps310_status, DPS310_SUCCESS, ERROR_BASE_DPS310); }

#endif /* DPS310_H */
