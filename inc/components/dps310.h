/*
 * dps310.h
 *
 *  Created on: 27 nov. 2018
 *      Author: Ludo
 */

#ifndef DPS310_H
#define DPS310_H

/*** DPS310 functions ***/

void DPS310_Init(void);
void DPS310_PerformMeasurements(void);
void DPS310_GetPressure(unsigned int* pressure_pa);
void DPS310_GetTemperature(signed char* temperature_degrees);

#endif /* DPS310_H */
