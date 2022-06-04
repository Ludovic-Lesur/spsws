/*
 * rain.h
 *
 *  Created on: 5 may 2019
 *      Author: Ludo
 */

#ifndef __RAIN_H__
#define __RAIN_H__

#include "gpio.h"
#include "mode.h"

#if (defined CM || defined ATM)

/*** RAIN functions ***/

void RAIN_init(void);
void RAIN_start_continuous_measure(void);
void RAIN_stop_continuous_measure(void);
void RAIN_get_pluviometry(unsigned char* rain_pluviometry_mm);
#ifdef FLOOD_DETECTION
void RAIN_get_flood_level(unsigned char* flood_level);
#endif
void RAIN_reset_data(void);
void RAIN_edge_callback(void);

#endif

#endif /* __RAIN_H__ */
