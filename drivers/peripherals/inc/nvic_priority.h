/*
 * nvic_priority.h
 *
 *  Created on: Aug 11, 2024
 *      Author: ludo
 */

#ifndef __NVIC_PRIORITY_H__
#define __NVIC_PRIORITY_H__

/*!******************************************************************
 * \enum NVIC_priority_list_t
 * \brief NVIC interrupt priorities list.
 *******************************************************************/
typedef enum {
	// Common.
	NVIC_PRIORITY_CLOCK = 0,
	NVIC_PRIORITY_CLOCK_CALIBRATION = 1,
	NVIC_PRIORITY_DELAY = 2,
	NVIC_PRIORITY_RTC = 3,
	// GPS.
	NVIC_PRIORITY_GPS_UART = 0,
	// DIOs.
	NVIC_PRIORITY_RAIN = 3,
	NVIC_PRIORITY_WIND = 3,
	// Sigfox.
	NVIC_PRIORITY_SIGFOX_MODULATION_GPIO = 0,
	NVIC_PRIORITY_SIGFOX_MODULATION_TIMER = 0,
	NVIC_PRIORITY_SIGFOX_TIMER = 1,
	// AT interface.
	NVIC_PRIORITY_AT = 3
} NVIC_priority_list_t;

#endif /* __NVIC_PRIORITY_H__ */
