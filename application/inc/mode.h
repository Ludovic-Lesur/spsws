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

#define SPSWS_WIND_RAINFALL_MEASUREMENTS

#if ((defined SPSWS_WIND_RAINFALL_MEASUREMENTS) && (defined HW2_0))
//#define SPSWS_SEN15901_EMULATOR
#endif

#endif /* __MODE_H__ */
