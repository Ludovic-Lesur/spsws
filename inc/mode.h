/*
 * mode.h
 *
 *  Created on: 22 dec. 2018
 *      Author: Ludo
 */

#ifndef MODE_H
#define MODE_H

/*** Wheather station mode ***/

// AT command mode.
#define ATM

// Intermittent mode with RTC as time reference.
//#define IM_RTC

// Continuous mode with RTC as time reference.
//#define CM_RTC

// Intermittent mode with hardware timer as time reference.
//#define IM_HWT

/*** Error management ***/

#if ((defined ATM && defined IM_RTC) || \
	 (defined ATM && defined CM_RTC) || \
	 (defined ATM && defined IM_HWT) || \
	 (defined IM_RTC && defined CM_RTC) || \
	 (defined IM_RTC && defined IM_HWT) || \
	 (defined CM_RTC && defined IM_HWT))
#error "Only one mode must be selected."
#endif

#endif /* MODE_H */
