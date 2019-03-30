/*
 * dlk.h
 *
 *  Created on: 30 march 2019
 *      Author: Ludovic
 */

#ifndef DLK_H
#define DLK_H

/*** DLK structures ***/

typedef struct {
	unsigned char dlk_local_utc_offset;
	unsigned char dlk_uplink_dlk_frame_enable;
	unsigned char dlk_geoloc_period_days;
	unsigned char dlk_weather_data_period_hours;
	unsigned char dlk_gps_timeout_seconds;
} DLK_Parameters;

/*** DLK functions ***/

void DLK_Read(DLK_Parameters* config);
void DLK_Write(unsigned char* downlink_data);

#endif /* DLK_H */
