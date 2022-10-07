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

typedef enum {
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
	SI1133_ERROR_BASE_I2C = 0x0100,
	SI1133_ERROR_BASE_LPTIM = (SI1133_ERROR_BASE_I2C + I2C_ERROR_BASE_LAST),
	SI1133_ERROR_BASE_LAST = (SI1133_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST)
} SI1133_status_t;

/*** SI1133 functions ***/

SI1133_status_t SI1133_perform_measurements(uint8_t si1133_i2c_address);
SI1133_status_t SI1133_get_uv_index(uint8_t* uv_index);

#define SI1133_status_check(error_base) { if (si1133_status != SI1133_SUCCESS) { status = error_base + si1133_status; goto errors; }}
#define SI1133_error_check() { ERROR_status_check(si1133_status, SI1133_SUCCESS, ERROR_BASE_SI1133); }
#define SI1133_error_check_print() { ERROR_status_check_print(si1133_status, SI1133_SUCCESS, ERROR_BASE_SI1133); }

#endif /* __SI1133_H__ */
