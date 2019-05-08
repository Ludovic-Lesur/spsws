/*
 * rain.h
 *
 *  Created on: 5 may 2019
 *      Author: Ludovic
 */

#ifndef RAIN_H
#define RAIN_H

#include "gpio.h"
#include "mode.h"

#if (defined CM || defined ATM)

/*** RAIN global variables ***/

GPIO GPIO_RAIN;

/*** RAIN functions ***/

void RAIN_Init(void);
void RAIN_StartContinuousMeasure(void);
void RAIN_StopContinuousMeasure(void);
void RAIN_GetPluviometry(unsigned char* rain_pluviometry_mm);
void RAIN_ResetData(void);

/*** RAIN utility functions ***/

void RAIN_EdgeCallback(void);

#endif

#endif /* RAIN_H */
