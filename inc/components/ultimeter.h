/*
 * ultimeter.h
 *
 *  Created on: 24 nov. 2018
 *      Author: Ludovic
 */

#ifndef ULTIMETER_H
#define ULTIMETER_H

/*** Ultimeter macros ***/

#define ULTIMETER_MEASUREMENT_PERIOD_SECONDS	2

/*** Ultimeter user functions ***/

void ULTIMETER_Init(void);
void ULTIMETER_StartContinuousMeasure(void);
void ULTIMETER_StopContinuousMeasure(void);
void ULTIMETER_GetAverageWindSpeed(unsigned char average_wind_speed_kmh);
void ULTIMETER_GetPeakWindSpeed(unsigned char peak_wind_speed_kmh);

/*** Ultimeter utility functions ***/

void ULTIMETER_IncrementWindSpeedCount(void);
void ULTIMETER_UpdateWindDirection(unsigned char new_wind_direction_pourcent);
void ULTIMETER_StoreMeasurements(void);

#endif /* ULTIMETER_H */
