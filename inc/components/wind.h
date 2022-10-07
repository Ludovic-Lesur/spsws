/*
 * wind.h
 *
 *  Created on: 24 nov. 2018
 *      Author: Ludo
 */

#ifndef __WIND_H__
#define __WIND_H__

#include "lptim.h"
#include "mapping.h"
#include "math.h"
#include "max11136.h"
#include "mode.h"
#include "spi.h"
#include "types.h"

/*** WIND macros ***/

#if (defined CM || defined ATM)
#define WIND_SPEED_MEASUREMENT_PERIOD_SECONDS		1
#define WIND_DIRECTION_MEASUREMENT_PERIOD_SECONDS	10
#endif

/*** WIND structures ***/

typedef enum {
	WIND_SUCCESS = 0,
	WIND_ERROR_NULL_PARAMETER,
	WIND_ERROR_MATH,
	WIND_ERROR_BASE_LPTIM = 0x0100,
	WIND_ERROR_BASE_SPI = (WIND_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
	WIND_ERROR_BASE_MAX11136 = (WIND_ERROR_BASE_SPI + SPI_ERROR_BASE_LAST),
	WIND_ERROR_BASE_MATH = (WIND_ERROR_BASE_MAX11136 + MAX11136_ERROR_BASE_LAST),
	WIND_ERROR_BASE_LAST = (WIND_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST)
} WIND_status_t;

/*** WIND functions ***/

#if (defined CM || defined ATM)
void WIND_init(void);
void WIND_start_continuous_measure(void);
void WIND_stop_continuous_measure(void);
WIND_status_t WIND_get_speed(uint32_t* average_speed_mh, uint32_t* peak_speed_mh);
WIND_status_t WIND_get_direction(uint32_t* average_direction_degrees);
void WIND_reset_data(void);
WIND_status_t WIND_speed_edge_callback(void);
#ifdef WIND_VANE_ULTIMETER
void WIND_direction_edge_callback(void);
#endif
WIND_status_t WIND_measurement_period_callback(void);
#endif

#define WIND_status_check(error_base) { if (wind_status != WIND_SUCCESS) { status = error_base + wind_status; goto errors; }}
#define WIND_error_check() { ERROR_status_check(wind_status, WIND_SUCCESS, ERROR_BASE_WIND); }
#define WIND_error_check_print() { ERROR_status_check(wind_status, WIND_SUCCESS, ERROR_BASE_WIND); }

#endif  /* __WIND_H__ */
