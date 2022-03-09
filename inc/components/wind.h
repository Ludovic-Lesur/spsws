/*
 * wind.h
 *
 *  Created on: 24 nov. 2018
 *      Author: Ludo
 */

#ifndef WIND_H
#define WIND_H

#include "gpio.h"
#include "lptim.h"
#include "mapping.h"
#include "max11136.h"
#include "mode.h"
#include "spi.h"

#if (defined CM || defined ATM)

/*** WIND macros ***/

#define WIND_SPEED_MEASUREMENT_PERIOD_SECONDS		1
#define WIND_DIRECTION_MEASUREMENT_PERIOD_SECONDS	10

/*** WIND structures ***/

typedef enum {
	WIND_SUCCESS = 0,
	WIND_ERROR_MATH,
	WIND_ERROR_BASE_LPTIM = 0x0100,
	WIND_ERROR_BASE_SPI = (WIND_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
	WIND_ERROR_BASE_MAX11136 = (WIND_ERROR_BASE_SPI + SPI_ERROR_BASE_LAST),
	WIND_ERROR_BASE_LAST = (WIND_ERROR_BASE_MAX11136 + MAX11136_ERROR_BASE_LAST)
} WIND_status_t;

/*** WIND functions ***/

void WIND_init(void);
void WIND_start_continuous_measure(void);
void WIND_stop_continuous_measure(void);
void WIND_get_speed(unsigned int* average_speed_mh, unsigned int* peak_speed_mh);
WIND_status_t WIND_get_direction(unsigned int* average_direction_degrees);
void WIND_reset_data(void);
WIND_status_t WIND_speed_edge_callback(void);
#ifdef WIND_VANE_ULTIMETER
void WIND_direction_edge_callback(void);
#endif
WIND_status_t WIND_measurement_period_callback(void);

#endif

#endif  /* WIND_H */
