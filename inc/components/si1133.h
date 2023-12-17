/*
 * si1133.h
 *
 *  Created on: 28 nov. 2018
 *      Author: Ludo
 */

#ifndef __SI1133_H__
#define __SI1133_H__

#include "i2c.h"
#include "types.h"

/*** SI1133 macros ***/

#define SI1133_EXTERNAL_I2C_ADDRESS		0x52
#define SI1133_ERROR_CODE_LAST			0x0F

/*** SI1133 structures ***/

/*!******************************************************************
 * \enum SI1133_status_t
 * \brief SI1133 driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	SI1133_SUCCESS = 0,
	SI1133_ERROR_NULL_PARAMETER,
	SI1133_ERROR_REGISTER_ADDRESS,
	SI1133_ERROR_REGISTER_BIT_INDEX,
	SI1133_ERROR_READY,
	SI1133_ERROR_COMMAND,
	SI1133_ERROR_COMMAND_COMPLETION,
	SI1133_ERROR_PARAMETER,
	SI1133_ERROR_PARAMETER_COMPLETION = (SI1133_ERROR_PARAMETER + SI1133_ERROR_CODE_LAST),
	SI1133_ERROR_COMMAND_COUNTER = (SI1133_ERROR_PARAMETER_COMPLETION + SI1133_ERROR_CODE_LAST),
	SI1133_ERROR_ACCESS_TYPE,
	SI1133_ERROR_TIMEOUT,
	// Low level drivers errors.
	SI1133_ERROR_BASE_I2C = 0x0100,
	SI1133_ERROR_BASE_LPTIM = (SI1133_ERROR_BASE_I2C + I2C_ERROR_BASE_LAST),
	// Last base value.
	SI1133_ERROR_BASE_LAST = (SI1133_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST)
} SI1133_status_t;

/*** SI1133 functions ***/

/*!******************************************************************
 * \fn SI1133_status_t SI1133_perform_measurements(uint8_t i2c_address)
 * \brief Perform UV index measurements.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SI1133_status_t SI1133_perform_measurements(uint8_t i2c_address);

/*!******************************************************************
 * \fn SI1133_status_t SI1133_get_uv_index(uint8_t* uv_index)
 * \brief Read UV index.
 * \param[in]  	none
 * \param[out] 	uv_index: Pointer to byte that will contain the UV index.
 * \retval		Function execution status.
 *******************************************************************/
SI1133_status_t SI1133_get_uv_index(uint8_t* uv_index);

/*******************************************************************/
#define SI1133_exit_error(error_base) { if (si1133_status != SI1133_SUCCESS) { status = (error_base + si1133_status); goto errors; } }

/*******************************************************************/
#define SI1133_stack_error(void) { if (si1133_status != SI1133_SUCCESS) { ERROR_stack_add(ERROR_BASE_SI1133 + si1133_status); } }

/*******************************************************************/
#define SI1133_stack_exit_error(error_code) { if (si1133_status != SI1133_SUCCESS) { ERROR_stack_add(ERROR_BASE_SI1133 + si1133_status); status = error_code; goto errors; } }

#endif /* __SI1133_H__ */
