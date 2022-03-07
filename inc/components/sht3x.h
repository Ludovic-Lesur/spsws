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

#define SHT3X_INTERNAL_I2C_ADDRESS	0x44
#define SHT3X_EXTERNAL_I2C_ADDRESS	0x45

/*** SHT3x structures ***/

typedef enum {
	SHT3X_SUCCESS = 0,
	SHT3X_ERROR_I2C,
	SHT3X_ERROR_LPTIM = (SHT3X_ERROR_I2C + I2C_ERROR_LAST),
	SHT3X_ERROR_LAST = (SHT3X_ERROR_LPTIM + LPTIM_ERROR_LAST),
} SHTX_status_t;

/*** SHT3x functions ***/

SHTX_status_t SHT3X_perform_measurements(unsigned char sht3x_i2c_address);
void SHT3X_get_temperature_comp1(unsigned char* temperature_degrees);
void SHT3X_get_temperature_comp2(signed char* temperature_degrees);
void SHT3X_get_humidity(unsigned char* humidity_percent);

#endif /* SHT3X_H */
