/*
 * si1133.h
 *
 *  Created on: 28 nov. 2018
 *      Author: Ludo
 */

#ifndef SI1133_H
#define SI1133_H

#include "i2c.h"

/*** SI1133 macros ***/

#define SI1133_EXTERNAL_I2C_ADDRESS		0x52

/*** SI1133 structures ***/

typedef enum {
	SI1133_SUCCESS = 0,
	SI1133_ERROR_READY,
	SI1133_ERROR_COMMAND,
	SI1133_ERROR_PARAMETER,
	SI1133_ERROR_TIMEOUT,
	SI1133_ERROR_BASE_I2C = 0x0100,
	SI1133_ERROR_BASE_LAST = (SI1133_ERROR_BASE_I2C + I2C_ERROR_BASE_LAST)
} SI1133_status_t;

/*** SI1133 functions ***/

SI1133_status_t SI1133_perform_measurements(unsigned char si1133_i2c_address);
void SI1133_get_uv_index(unsigned char* uv_index);

#endif /* SI1133_H */
