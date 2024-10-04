/*
 * embedded_utils_flags.h
 *
 *  Created on: 15 aug. 2024
 *      Author: Ludo
 */

#ifndef __EMBEDDED_UTILS_FLAGS_H__
#define __EMBEDDED_UTILS_FLAGS_H__

#include "mode.h"

/*** Embedded utility functions compilation flags ***/

#define EMBEDDED_UTILS_ERROR_STACK_DEPTH			32
#define EMBEDDED_UTILS_ERROR_STACK_SUCCESS_VALUE	0
#define EMBEDDED_UTILS_ERROR_STACK_SIGFOX

#ifdef HW1_0
#define EMBEDDED_UTILS_MATH_PRECISION				0
#endif
#ifdef HW2_0
#define EMBEDDED_UTILS_MATH_PRECISION				1
#endif
#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
#define EMBEDDED_UTILS_MATH_COS_TABLE
#define EMBEDDED_UTILS_MATH_SIN_TABLE
#define EMBEDDED_UTILS_MATH_ATAN2
#endif

#endif /* __EMBEDDED_UTILS_FLAGS_H__ */
