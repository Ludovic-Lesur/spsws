/*
 * geoloc.h
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludo
 */

#ifndef GEOLOC_H
#define GEOLOC_H

#include "neom8n.h"

/*** GEOLOC macros ***/

// Length of geolocation Sigfox frame in bytes.
#define GEOLOC_SIGFOX_DATA_LENGTH	11

/*** GEOLOC functions ***/

void GEOLOC_BuildSigfoxData(Position* geoloc_position, unsigned char geoloc_fix_duration_seconds, unsigned char* geoloc_sigfox_data);

#endif /* GEOLOC_H */
