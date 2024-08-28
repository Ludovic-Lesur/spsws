/*
 * dps310.h
 *
 *  Created on: 27 nov. 2018
 *      Author: Ludo
 */

#ifndef __DPS310_H__
#define __DPS310_H__

#include "i2c.h"
#include "lptim.h"
#include "math.h"
#include "types.h"

/*** DPS310 structures ***/

/*!******************************************************************
 * \enum DPS310_status_t
 * \brief DPS310 driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	DPS310_SUCCESS = 0,
	DPS310_ERROR_NULL_PARAMETER,
	DPS310_ERROR_REGISTER_ADDRESS,
	DPS310_ERROR_REGISTER_BIT_INDEX,
	DPS310_ERROR_COEFFICIENTS_TIMEOUT,
	DPS310_ERROR_SENSOR_TIMEOUT,
	DPS310_ERROR_TEMPERATURE_TIMEOUT,
	DPS310_ERROR_PRESSURE_TIMEOUT,
	// Low level drivers errors.
	DPS310_ERROR_BASE_I2C = 0x0100,
	DPS310_ERROR_BASE_LPTIM = (DPS310_ERROR_BASE_I2C + I2C_ERROR_BASE_LAST),
	DPS310_ERROR_BASE_MATH = (DPS310_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
	// Last base value.
	DPS310_ERROR_BASE_LAST = (DPS310_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST)
} DPS310_status_t;

/*** DPS310 functions ***/

/*!******************************************************************
 * \fn DPS310_status_t DPS310_init(void)
 * \brief Init DPS310 driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
DPS310_status_t DPS310_init(void);

/*!******************************************************************
 * \fn DPS310_status_t DPS310_de_init(void)
 * \brief Release DPS310 driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
DPS310_status_t DPS310_de_init(void);

/*!******************************************************************
 * \fn DPS310_status_t DPS310_perform_measurements(uint8_t i2c_address)
 * \brief Perform pressure and temperature measurements.
 * \param[in]  	i2c_address: I2C address of the sensor.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
DPS310_status_t DPS310_perform_measurements(uint8_t i2c_address);

/*!******************************************************************
 * \fn DPS310_status_t DPS310_get_pressure(int32_t* pressure_pa)
 * \brief Read pressure.
 * \param[in]  	none
 * \param[out] 	pressure_pa: Pointer to integer that will contain the pressure in Pa.
 * \retval		Function execution status.
 *******************************************************************/
DPS310_status_t DPS310_get_pressure(int32_t* pressure_pa);

/*!******************************************************************
 * \fn DPS310_status_t DPS310_get_temperature(int32_t* temperature_degrees)
 * \brief Read temperature.
 * \param[in]  	none
 * \param[out] 	temperature_degrees: Pointer to integer that will contain the temperature in degrees.
 * \retval		Function execution status.
 *******************************************************************/
DPS310_status_t DPS310_get_temperature(int32_t* temperature_degrees);

/*******************************************************************/
#define DPS310_exit_error(error_base) { if (dps310_status != DPS310_SUCCESS) { status = (error_base + dps310_status); goto errors; } }

/*******************************************************************/
#define DPS310_stack_error(void) { if (dps310_status != DPS310_SUCCESS) { ERROR_stack_add(ERROR_BASE_DPS310 + dps310_status); } }

/*******************************************************************/
#define DPS310_stack_exit_error(error_code) { if (dps310_status != DPS310_SUCCESS) { ERROR_stack_add(ERROR_BASE_DPS310 + dps310_status); status = error_code; goto errors; } }

#endif /* __DPS310_H__ */
