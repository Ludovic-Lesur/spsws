/*
 * wind.h
 *
 *  Created on: 24 nov. 2018
 *      Author: Ludo
 */

#ifndef WIND_H
#define WIND_H

#include "mode.h"

#if (defined CM_RTC || defined ATM)

/*** WIND macros ***/

#define WIND_MEASUREMENT_PERIOD_SECONDS		4

/*** WIND functions ***/

void WIND_Init(void);
void WIND_StartContinuousMeasure(void);
void WIND_StopContinuousMeasure(void);
void WIND_GetSpeed(unsigned int* average_wind_speed_mh, unsigned int* peak_wind_speed_mh);
void WIND_GetDirection(unsigned int* average_wind_direction_degrees);
void WIND_ResetData(void);

/*** Wind utility functions ***/

void WIND_MeasurementPeriodCallback(void);
void WIND_SpeedEdgeCallback(void);
#ifdef WIND_VANE_ULTIMETER
void WIND_DirectionEdgeCallback(void);
#endif

#endif /* WIND_H */

#endif
