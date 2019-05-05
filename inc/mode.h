/*
 * mode.h
 *
 *  Created on: 22 dec. 2018
 *      Author: Ludo
 */

#ifndef MODE_H
#define MODE_H

#include "sigfox_api.h"

/*** Wheather station mode ***/

//#define ATM 		// AT command mode.
#define IM_RTC 		// Intermittent mode with RTC as time reference.
//#define CM_RTC 	// Continuous mode with RTC as time reference.
//#define IM_HWT 	// Intermittent mode with hardware timer as time reference.

/*** Sigfox RC configuration ***/

#define SPSWS_SIGFOX_RC		RC1

/*** Debug mode ***/

#define DEBUG		// Use LED and programming pins for debug purpose.

/*** Error management ***/

#if ((defined ATM && defined IM_RTC) || \
	 (defined ATM && defined CM_RTC) || \
	 (defined ATM && defined IM_HWT) || \
	 (defined IM_RTC && defined CM_RTC) || \
	 (defined IM_RTC && defined IM_HWT) || \
	 (defined CM_RTC && defined IM_HWT))
#error "Only 1 weather station mode must be selected."
#endif

#endif /* MODE_H */
