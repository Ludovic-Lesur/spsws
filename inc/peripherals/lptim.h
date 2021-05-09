/*
 * lptim.h
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludo
 */

#ifndef LPTIM_H
#define LPTIM_H

#include "wind.h"

/*** LPTIM functions ***/

void LPTIM1_Init(unsigned int lsi_freq_hz);
void LPTIM1_Enable(void);
void LPTIM1_Disable(void);

void LPTIM1_DelayMilliseconds(unsigned int delay_ms, unsigned char stop_mode);

#ifdef WIND_VANE_ULTIMETER
void LPTIM1_Start(void);
void LPTIM1_Stop(void);
unsigned int LPTIM1_GetCounter(void);
#endif

#endif /* LPTIM_H */
