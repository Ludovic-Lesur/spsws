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

#define WIND_MEASUREMENT_PERIOD_SECONDS		4
// Wind vane type.
//#define WIND_VANE_ULTIMETER				// Phase shift technique.
#define WIND_VANE_ARGENT_DATA_SYSTEMS		// Analog technique.

/*** WIND global variables ***/

GPIO GPIO_WIND_SPEED;
#ifdef WIND_VANE_ULTIMETER
GPIO GPIO_WIND_DIRECTION;
#endif

/*** WIND functions ***/

void WIND_Init(void);
void WIND_StartContinuousMeasure(void);
void WIND_StopContinuousMeasure(void);
void WIND_GetSpeed(unsigned int* average_wind_speed_mh, unsigned int* peak_wind_speed_mh);
void WIND_GetDirection(unsigned int* average_wind_direction_degrees);
void WIND_ResetData(void);

/*** WIND utility functions ***/

void WIND_MeasurementPeriodCallback(void);
void WIND_SpeedEdgeCallback(void);
#ifdef WIND_VANE_ULTIMETER
void WIND_DirectionEdgeCallback(void);
#endif

/*** Errors management ***/

#if (defined WIND_VANE_ULTIMETER && defined WIND_VANE_ARGENT_DATA_SYSTEMS)
#error "Only one wind vane type must be selected"
#endif

#endif /* WIND_H */

#endif
