/*
 * wind.h
 *
 *  Created on: 24 nov. 2018
 *      Author: Ludo
 */

#ifndef WIND_H
#define WIND_H

/*** WIND macros ***/

#define WIND_MEASUREMENT_PERIOD_SECONDS		2

/*** WIND functions ***/

void WIND_Init(void);
void WIND_StartContinuousMeasure(void);
void WIND_StopContinuousMeasure(void);
void WIND_GetAveragePeakWindSpeed(unsigned char* average_wind_speed_kmh, unsigned char* peak_wind_speed_kmh);
void WIND_GetAverageWindDirection(unsigned char* average_wind_direction_pourcent);

#endif /* WIND_H */
