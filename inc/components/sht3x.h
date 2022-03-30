/*
 * sht3x.h
 *
 *  Created on: 27 oct. 2018
 *      Author: Ludo
 */

#ifndef SHT3X_H
#define SHT3X_H

#include "i2c.h"
#include "lptim.h"

/*** SHT3x macros ***/

#define SHT3X_INT_I2C_ADDRESS	0x44
#define SHT3X_EXT_I2C_ADDRESS	0x45

/*** SHT3x structures ***/

typedef enum {
	SHT3X_SUCCESS = 0,
	SHT3X_ERROR_BASE_I2C = 0x0100,
	SHT3X_ERROR_BASE_LPTIM = (SHT3X_ERROR_BASE_I2C + I2C_ERROR_BASE_LAST),
	SHT3X_ERROR_BASE_LAST = (SHT3X_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST)
} SHT3X_status_t;

/*** SHT3x functions ***/

SHT3X_status_t SHT3X_perform_measurements(unsigned char sht3x_i2c_address);
void SHT3X_get_temperature(signed char* temperature_degrees);
void SHT3X_get_humidity(unsigned char* humidity_percent);

#define SHT3X_INT_status_check(error_base) { if (sht3x_status != SHT3X_SUCCESS) { status = error_base + sht3x_status; goto errors; }}
#define SHT3X_INT_error_check() { ERROR_status_check(sht3x_status, SHT3X_SUCCESS, ERROR_BASE_SHT3X_INT); }
#define SHT3X_INT_error_check_print() { ERROR_status_check_print(sht3x_status, SHT3X_SUCCESS, ERROR_BASE_SHT3X_INT); }

#define SHT3X_EXT_status_check(error_base) { if (sht3x_status != SHT3X_SUCCESS) { status = error_base + sht3x_status; goto errors; }}
#define SHT3X_EXT_error_check() { ERROR_status_check(sht3x_status, SHT3X_SUCCESS, ERROR_BASE_SHT3X_EXT); }
#define SHT3X_EXT_error_check_print() { ERROR_status_check_print(sht3x_status, SHT3X_SUCCESS, ERROR_BASE_SHT3X_EXT); }

#endif /* SHT3X_H */
