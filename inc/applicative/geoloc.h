/*
 * geoloc.h
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludo
 */

#ifndef GEOLOC_H
#define GEOLOC_H

#include "neom8n.h"

/*** GEOLOC functions ***/

void GEOLOC_Process(Timestamp* gps_timestamp, unsigned char* timestamp_retrieved);

#endif /* GEOLOC_H */
