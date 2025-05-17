/*
 * spsws_flags.h
 *
 *  Created on: 22 dec. 2018
 *      Author: Ludo
 */

#ifndef __SPSWS_FLAGS_H__
#define __SPSWS_FLAGS_H__

/*** Board modes ***/

//#define SPSWS_MODE_CLI
//#define SPSWS_MODE_DEBUG

/*** Board options ***/

#define SPSWS_WIND_RAINFALL_MEASUREMENTS

//#define SPSWS_WIND_VANE_ULTIMETER

#if ((defined SPSWS_WIND_RAINFALL_MEASUREMENTS) && (defined HW2_0))
//#define SPSWS_SEN15901_EMULATOR
#endif

#endif /* __SPSWS_FLAGS_H__ */
