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

void LPTIM1_init(unsigned int lsi_freq_hz);
void LPTIM1_enable(void);
void LPTIM1_disable(void);
void LPTIM1_delay_milliseconds(unsigned int delay_ms, unsigned char stop_mode);
#ifdef WIND_VANE_ULTIMETER
void LPTIM1_start(void);
void LPTIM1_stop(void);
unsigned int LPTIM1_get_counter(void);
#endif

#endif /* LPTIM_H */
