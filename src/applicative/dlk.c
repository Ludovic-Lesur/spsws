/*
 * dlk.c
 *
 *  Created on: 30 march 2019
 *      Author: Ludovic
 */

#include "dlk.h"

#include "nvm.h"
#include "sigfox_api.h"

/*** DLK local macros ***/

#define DLK_UPLINK_CONFIG_FRAME_ENABLE_BIT_IDX	7
#define DLK_GEOLOCATION_PERIOD_BIT_IDX			2
#define DLK_WEATHER_DATA_PERIOD_BIT_IDX			0

/*** DLK local structures ***/

typedef enum {
	DLK_GEOLOC_PERIOD_1_DAY,
	DLK_GEOLOC_PERIOD_2_DAYS,
	DLK_GEOLOC_PERIOD_4_DAYS,
	DLK_GEOLOC_PERIOD_7_DAYS
} DLK_GeolocPeriod;

typedef enum {
	DLK_WEATHER_DATA_PERIOD_1_HOUR,
	DLK_WEATHER_DATA_PERIOD_2_HOURS,
	DLK_WEATHER_DATA_PERIOD_4_HOURS,
	DLK_WEATHER_DATA_PERIOD_6_HOURS,
} DLK_WeatherDataPeriod;

/* READ THE CURRENT CONFIGURATION FROM NVM.
 * @param config:	Pointer to DLK_Parameters strcutres that will contain current configuration.
 * @return:			None.
 */
void DLK_Read(DLK_Parameters* config) {

	/* Local UTC offset */
	NVM_ReadByte(NVM_CONFIG_LOCAL_UTC_OFFSET_ADDRESS_OFFSET, &(config -> dlk_local_utc_offset));

	/* Uplink configuration frame enable */
	unsigned char nvm_byte = 0;
	NVM_ReadByte(NVM_CONFIG_UPLINK_FRAMES_ADDRESS_OFFSET, &nvm_byte);
	config -> dlk_uplink_dlk_frame_enable = (nvm_byte & (0b1 << DLK_UPLINK_CONFIG_FRAME_ENABLE_BIT_IDX)) >> DLK_UPLINK_CONFIG_FRAME_ENABLE_BIT_IDX;

	/* Geolocation period */
	switch ((nvm_byte & (0b11 << DLK_GEOLOCATION_PERIOD_BIT_IDX)) >> DLK_GEOLOCATION_PERIOD_BIT_IDX) {
	case DLK_GEOLOC_PERIOD_1_DAY:
		config -> dlk_geoloc_period_days = 1;
		break;
	case DLK_GEOLOC_PERIOD_2_DAYS:
		config -> dlk_geoloc_period_days = 2;
		break;
	case DLK_GEOLOC_PERIOD_4_DAYS:
		config -> dlk_geoloc_period_days = 4;
		break;
	case DLK_GEOLOC_PERIOD_7_DAYS:
		config -> dlk_geoloc_period_days = 7;
		break;
	}

	/* Weather data period */
	switch ((nvm_byte & (0b11 << DLK_WEATHER_DATA_PERIOD_BIT_IDX)) >> DLK_WEATHER_DATA_PERIOD_BIT_IDX) {
	case DLK_WEATHER_DATA_PERIOD_1_HOUR:
		config -> dlk_weather_data_period_hours = 1;
		break;
	case DLK_WEATHER_DATA_PERIOD_2_HOURS:
		config -> dlk_weather_data_period_hours = 2;
		break;
	case DLK_WEATHER_DATA_PERIOD_4_HOURS:
		config -> dlk_weather_data_period_hours = 4;
		break;
	case DLK_WEATHER_DATA_PERIOD_6_HOURS:
		config -> dlk_weather_data_period_hours = 6;
		break;
	}

	/* GPS timeout */
	NVM_ReadByte(NVM_CONFIG_GPS_TIMEOUT_ADDRESS_OFFSET, &(config -> dlk_gps_timeout_seconds));
}

/* WRITE A NEW CONFIGURATION FROM NVM.
 * @param downlink_data:	Raw downlink received from Sigfox network.
 * @return:					None.
 */
void DLK_Write(unsigned char* downlink_data) {
	// Fill NVM with new data.
	unsigned char byte_idx = 0;
	for (byte_idx=0 ; byte_idx<SFX_DOWNLINK_DATA_SIZE_BYTES ; byte_idx++) {
		NVM_WriteByte((NVM_CONFIG_START_ADDRESS_OFFSET + byte_idx), downlink_data[byte_idx]);
	}
}
