/*
 * geoloc.c
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludo
 */

#include "geoloc.h"

#include "neom8n.h"

/*** GEOLOC functions ***/

/* BUILD SIGFOX DATA STARTING FROM GPS POSITION AND STATUS.
 * @param:	None.
 * @return:	None.
 */
void GEOLOC_BuildSigfoxData(Position* geoloc_position, unsigned char geoloc_fix_duration_seconds, unsigned char* geoloc_sigfox_data) {
	// Latitude degrees.
	geoloc_sigfox_data[0] = geoloc_position -> lat_degrees;
	// Latitude minutes.
	geoloc_position -> lat_minutes = (geoloc_position -> lat_minutes) & 0x3F; // Ensure minutes are on 6 bits.
	geoloc_sigfox_data[1] = (geoloc_position -> lat_minutes) << 2;
	// Latitude seconds.
	geoloc_position -> lat_seconds = (geoloc_position -> lat_seconds) & 0x0001FFFF; // Ensure seconds are on 17 bits.
	geoloc_sigfox_data[1] |= (((geoloc_position -> lat_seconds) & 0x00018000) >> 15); // Keep bits 15-16.
	geoloc_sigfox_data[2] = (((geoloc_position -> lat_seconds) & 0x00007F80) >> 7); // Keep bits 7-14.
	geoloc_sigfox_data[3] = (((geoloc_position -> lat_seconds) & 0x0000007F) << 1); // Keep bits 0-6.
	// Latitude N/S.
	geoloc_position -> lat_north = (geoloc_position -> lat_north) & 0x01; // Ensure north flag is a bit.
	geoloc_sigfox_data[3] |= geoloc_position -> lat_north;
	// Longitude degrees.
	geoloc_sigfox_data[4] = geoloc_position -> long_degrees;
	// Longitude minutes.
	geoloc_position -> long_minutes = (geoloc_position -> long_minutes) & 0x3F; // Ensure minutes are on 6 bits.
	geoloc_sigfox_data[5] = (geoloc_position -> long_minutes) << 2;
	// Longitude seconds.
	geoloc_position -> long_seconds = (geoloc_position -> long_seconds) & 0x0001FFFF; // Ensure seconds are on 17 bits.
	geoloc_sigfox_data[5] |= (((geoloc_position -> long_seconds) & 0x00018000) >> 15); // Keep bits 15-16.
	geoloc_sigfox_data[6] = (((geoloc_position -> long_seconds) & 0x00007F80) >> 7); // Keep bits 7-14.
	geoloc_sigfox_data[7] = (((geoloc_position -> long_seconds) & 0x0000007F) << 1); // Keep bits 0-6.
	// Longitude E/O.
	geoloc_position -> long_east = (geoloc_position -> long_east) & 0x01; // Ensure east flag is a bit.
	geoloc_sigfox_data[7] |= geoloc_position -> long_east;
	// Altitude in m.
	geoloc_position -> altitude = (geoloc_position -> altitude) & 0x0000FFFF; // Ensure altitude is on 16 bits.
	geoloc_sigfox_data[8] = (((geoloc_position -> altitude) & 0x0000FF00) >> 8);
	geoloc_sigfox_data[9] = ((geoloc_position -> altitude) & 0x000000FF);
	// Fix duration.
	geoloc_sigfox_data[10] = geoloc_fix_duration_seconds;
}
