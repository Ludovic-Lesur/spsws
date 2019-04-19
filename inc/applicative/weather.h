/*
 * weather.h
 *
 *  Created on: 30 march 2019
 *      Author: Ludovic
 */

#ifndef WEATHER_H
#define WEATHER_H

#include "mode.h"

/*** WEATHER macros ***/

// Length of weather data Sigfox frame in bytes.
#ifdef CM_RTC
#define WEATHER_SIGFOX_DATA_LENGTH		10
#else
#define WEATHER_SIGFOX_DATA_LENGTH		6
#endif

/*** WEATHER structures ***/

typedef struct {
	signed char weather_data_temperature_degrees;
	unsigned char weather_data_humidity_percent;
	unsigned char weather_data_light_percent;
	unsigned char weather_data_uv_index;
	unsigned int weather_data_pressure_pa;
#ifdef CM_RTC
	unsigned char weather_data_average_wind_speed_kmh;
	unsigned char weather_data_peak_wind_speed_kmh;
	unsigned char weather_data_average_wind_direction_degrees;
	unsigned char weather_data_rain_mm;
#endif
} WEATHER_Data;

/*** WEATHER functions ***/

void WEATHER_BuildSigfoxData(WEATHER_Data* weather_data, unsigned char* weather_sigfox_data);

#endif /* WEATHER_H */
