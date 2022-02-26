/*
 * wind.h
 *
 *  Created on: 24 nov. 2018
 *      Author: Ludo
 */

#ifndef WIND_H
#define WIND_H

#include "gpio.h"
#include "mapping.h"
#include "mode.h"

#if (defined CM || defined ATM)

/*** WIND macros ***/

#define WIND_SPEED_MEASUREMENT_PERIOD_SECONDS		1
#define WIND_DIRECTION_MEASUREMENT_PERIOD_SECONDS	10
// Wind vane type.
//#define WIND_VANE_ULTIMETER						// Phase shift technique.
#define WIND_VANE_ARGENT_DATA_SYSTEMS				// Analog technique.

#define WIND_DIRECTION_ERROR_VALUE					0xFFFFFFFF

/*** WIND global variables ***/

GPIO_pin_t GPIO_WIND_SPEED;
#ifdef WIND_VANE_ULTIMETER
GPIO_pin_t GPIO_WIND_DIRECTION;
#endif

/*** WIND functions ***/

void WIND_init(void);
void WIND_start_continuous_measure(void);
void WIND_stop_continuous_measure(void);
void WIND_get_speed(unsigned int* average_speed_mh, unsigned int* peak_speed_mh);
void WIND_get_direction(unsigned int* average_direction_degrees);
void WIND_reset_data(void);
void WIND_speed_edge_callback(void);
#ifdef WIND_VANE_ULTIMETER
void WIND_direction_edge_callback(void);
#endif
void WIND_measurement_period_callback(void);

/*** Errors management ***/

#if (defined WIND_VANE_ULTIMETER && defined WIND_VANE_ARGENT_DATA_SYSTEMS)
#error "Only one wind vane type must be selected"
#endif

#endif /* WIND_H */

#endif
