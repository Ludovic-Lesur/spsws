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

/*** RAIN functions ***/

void RAIN_init(void);
void RAIN_start_continuous_measure(void);
void RAIN_stop_continuous_measure(void);
void RAIN_get_pluviometry(unsigned char* rain_pluviometry_mm);
void RAIN_reset_data(void);
void RAIN_edge_callback(void);

#endif

#endif /* RAIN_H */
