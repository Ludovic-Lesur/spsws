/*
 * mode.h
 *
 *  Created on: 22 dec. 2018
 *      Author: Ludo
 */

#ifndef __MODE_H__
#define __MODE_H__

/*** Board modes ***/

//#define ATM
//#define DEBUG

/*** Board options ***/

// Measurement features.
#define SPSWS_WIND_MEASUREMENT
#define SPSWS_RAIN_MEASUREMENT
#if (defined SPSWS_RAIN_MEASUREMENT) && (defined HW2_0)
//#define SPSWS_FLOOD_MEASUREMENT
#endif

// Wind vane type.
#ifdef SPSWS_WIND_MEASUREMENT
//#define WIND_VANE_ULTIMETER
#define WIND_VANE_ARGENT_DATA_SYSTEMS
#endif

/*** Errors management ***/

#if (defined WIND_VANE_ULTIMETER && defined WIND_VANE_ARGENT_DATA_SYSTEMS)
#error "Only one wind vane type must be selected"
#endif

#endif /* __MODE_H__ */
