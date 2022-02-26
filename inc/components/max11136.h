/*
 * max11136.h
 *
 *  Created on: 17 nov. 2018
 *      Author: Ludo
 */

#ifndef MAX11136_H
#define MAX11136_H

#include "wind.h"

/*** MAX11136 macros ***/

#define MAX11136_NUMBER_OF_CHANNELS			8
#define MAX11136_FULL_SCALE					4095

// Channels mapping.
#ifdef HW1_0
#if (defined CM || defined ATM)
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
#define MAX11136_CHANNEL_WIND_DIRECTION		0
#endif
#endif
#define MAX11136_CHANNEL_SOLAR_CELL			4
#define MAX11136_CHANNEL_SUPERCAP			5
#define MAX11136_CHANNEL_LDR				6
#define MAX11136_CHANNEL_BANDGAP			7
#define MAX11136_BANDGAP_VOLTAGE_MV			2048
#endif
#ifdef HW2_0
#if (defined CM || defined ATM)
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
#define MAX11136_CHANNEL_WIND_DIRECTION		0
#endif
#endif
#define MAX11136_CHANNEL_LDR				1
#define MAX11136_CHANNEL_BANDGAP			5
#define MAX11136_BANDGAP_VOLTAGE_MV			2048
#define MAX11136_CHANNEL_SOLAR_CELL			6
#define MAX11136_CHANNEL_SUPERCAP			7
#endif

/*** MAX11136 functions ***/

void MAX11136_init(void);
void MAX11136_disable_gpio(void);
void MAX11136_perform_measurements(void);
void MAX11136_get_data(unsigned char channel, unsigned int* channel_result_12bits);

#endif /* MAX11136_H */
