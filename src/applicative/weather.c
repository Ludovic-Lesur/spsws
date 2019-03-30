/*
 * weather.c
 *
 *  Created on: 30 march 2019
 *      Author: Ludovic
 */

#include "weather.h"

#include "mode.h"

/*** WEATHER functions ***/

/* BUILD SIGFOX UPLINK WEATHER DATA FRAME.
 * @param weather_data:			Raw data structure retrieved from sensors.
 * @param weather_sigfox_data:	Byte array that will contain Sigfox data.
 * @return:						None.
 */
void WEATHER_BuildSigfoxData(WEATHER_Data* weather_data, unsigned char* weather_sigfox_data) {
	// Temperature (°C)
	weather_sigfox_data[0] = weather_data -> weather_data_temperature_degrees;
	// Humidity (%).
	weather_sigfox_data[1] = weather_data -> weather_data_humidity_percent;
	// Light (%).
	weather_sigfox_data[2] = weather_data -> weather_data_light_percent;
	// UV index.
	weather_sigfox_data[3] = weather_data -> weather_data_uv_index;
	// Absolute presssure (1/10 hPa).
	weather_sigfox_data[4] = (((weather_data -> weather_data_pressure_pa) / (10)) & 0xFF00) >> 8;
	weather_sigfox_data[5] = (((weather_data -> weather_data_pressure_pa) / (10)) & 0x00FF) >> 0;
#ifdef CM_RTC
	// Average wind speed.
	weather_sigfox_data[6] = weather_data -> weather_data_average_wind_speed_kmh;
	// Peak wind speed.
	weather_sigfox_data[7] = weather_data -> weather_data_peak_wind_speed_kmh;
	// Average wind direction.
	weather_sigfox_data[8] = weather_data -> weather_data_average_wind_direction_kmh;
	// Rain.
	weather_sigfox_data[9] = weather_data -> weather_data_peak_wind_speed_kmh;
	// Status byte.
	weather_sigfox_data[10] = weather_data -> weather_data_status_byte;
#else
	weather_sigfox_data[6] = weather_data -> weather_data_status_byte;
#endif
}

