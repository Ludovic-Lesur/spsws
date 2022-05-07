/*
 * mode.h
 *
 *  Created on: 22 dec. 2018
 *      Author: Ludo
 */

#ifndef MODE_H
#define MODE_H

/*** Wheather station mode ***/

//#define ATM 		// AT command mode.
#define IM 			// Intermittent mode.
//#define CM 		// Continuous mode.

#ifdef IM
#define SPSWS_MODE	0
#else
#define SPSWS_MODE	1
#endif

/*** Wind vane selection ***/

#if (defined CM || defined ATM)
// Wind vane type.
//#define WIND_VANE_ULTIMETER			// Phase shift technique.
#define WIND_VANE_ARGENT_DATA_SYSTEMS	// Analog technique.
#endif

/*** Flood detection feature ***/

#if (defined HW2_0) && (defined CM) && (defined WIND_VANE_ARGENT_DATA_SYSTEMS)
//#define FLOOD_DETECTION
#endif

/*** Debug mode ***/

//#define DEBUG		// Use LED and programming pins for debug purpose if defined.

/*** Error management ***/

#if ((defined ATM && defined IM) || \
	 (defined ATM && defined CM) || \
	 (defined IM && defined CM))
#error "Only 1 weather station mode must be selected."
#endif

#if (defined WIND_VANE_ULTIMETER && defined WIND_VANE_ARGENT_DATA_SYSTEMS)
#error "Only one wind vane type must be selected"
#endif

#endif /* MODE_H */
