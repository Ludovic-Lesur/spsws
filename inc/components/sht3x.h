/*
 * sht3x.h
 *
 *  Created on: 27 oct. 2018
 *      Author: Ludo
 */

#ifndef __SHT3X_H__
#define __SHT3X_H__

#include "i2c.h"
#include "lptim.h"
#include "types.h"

/*** SHT3x macros ***/

#define SHT3X_INT_I2C_ADDRESS	0x44
#define SHT3X_EXT_I2C_ADDRESS	0x45

/*** SHT3x structures ***/

/*!******************************************************************
 * \enum SHT3X_status_t
 * \brief SHT3X driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	SHT3X_SUCCESS = 0,
	SHT3X_ERROR_NULL_PARAMETER,
	// Low level drivers errors.
	SHT3X_ERROR_BASE_I2C1 = 0x0100,
	SHT3X_ERROR_BASE_LPTIM1 = (SHT3X_ERROR_BASE_I2C1 + I2C_ERROR_BASE_LAST),
	// Last base value.
	SHT3X_ERROR_BASE_LAST = (SHT3X_ERROR_BASE_LPTIM1 + LPTIM_ERROR_BASE_LAST)
} SHT3X_status_t;

/*** SHT3x functions ***/

/*!******************************************************************
 * \fn SHT3X_status_t SHT3X_perform_measurements(uint8_t i2c_address)
 * \brief Perform temperature and humidity measurements.
 * \param[in]  	i2c_address: I2C address of the sensor.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SHT3X_status_t SHT3X_perform_measurements(uint8_t i2c_address);

/*!******************************************************************
 * \fn SHT3X_status_t SHT3X_get_temperature(int8_t* temperature_degrees)
 * \brief Read temperature.
 * \param[in]  	none
 * \param[out] 	temperature_degrees: Pointer to signed byte that will contain the temperature in degrees.
 * \retval		Function execution status.
 *******************************************************************/
SHT3X_status_t SHT3X_get_temperature(int8_t* temperature_degrees);

/*!******************************************************************
 * \fn SHT3X_status_t SHT3X_get_humidity(uint8_t* humidity_percent)
 * \brief Read humidity.
 * \param[in]  	none
 * \param[out] 	humidity_percent: Pointer to byte that will contain the humidity in percent.
 * \retval		Function execution status.
 *******************************************************************/
SHT3X_status_t SHT3X_get_humidity(uint8_t* humidity_percent);

/*******************************************************************/
#define SHT3X_exit_error(error_base) { if (sht3x_status != SHT3X_SUCCESS) { status = (error_base + sht3x_status); goto errors; } }

/*******************************************************************/
#define SHT3X_stack_error(void) { if (sht3x_status != SHT3X_SUCCESS) { ERROR_stack_add(ERROR_BASE_SHT3X + sht3x_status); } }

/*******************************************************************/
#define SHT3X_stack_exit_error(error_code) { if (sht3x_status != SHT3X_SUCCESS) { ERROR_stack_add(ERROR_BASE_SHT3X + sht3x_status); status = error_code; goto errors; } }

#endif /* __SHT3X_H__ */
