/*
 * nvic_priority.h
 *
 *  Created on: 11 aug. 2024
 *      Author: Ludo
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
    NVIC_PRIORITY_RAINFALL = 3,
    NVIC_PRIORITY_WIND_SPEED = 2,
    // Sigfox.
    NVIC_PRIORITY_SIGFOX_MODULATION_GPIO = 0,
    NVIC_PRIORITY_SIGFOX_MODULATION_TIMER = 0,
    NVIC_PRIORITY_SIGFOX_TIMER = 1,
    // AT interface.
    NVIC_PRIORITY_AT = 3
} NVIC_priority_list_t;

#endif /* __NVIC_PRIORITY_H__ */
