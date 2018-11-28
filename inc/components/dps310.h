/*
 * dps310.h
 *
 *  Created on: 27 nov. 2018
 *      Author: Ludovic
 */

#ifndef DPS310_H
#define DPS310_H

/*** DPS310 functions ***/

void DPS310_GetTemperature(signed char* temperature_degrees);
void DPS310_GetPressure(unsigned int* pressure_pa);

#endif /* DPS310_H */
