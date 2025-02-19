/*
 * sen15901_driver_flags.h
 *
 *  Created on: 07 sep. 2024
 *      Author: Ludo
 */

#ifndef __SEN15901_DRIVER_FLAGS_H__
#define __SEN15901_DRIVER_FLAGS_H__

#include "analog.h"
#include "spsws_flags.h"

/*** SEN15901 driver compilation flags ***/

#ifndef SPSWS_WIND_RAINFALL_MEASUREMENTS
#define SEN15901_DRIVER_DISABLE
#endif

#define SEN15901_DRIVER_GPIO_ERROR_BASE_LAST                    0
#define SEN15901_DRIVER_TIMER_ERROR_BASE_LAST                   0
#define SEN15901_DRIVER_ADC_ERROR_BASE_LAST                     ANALOG_ERROR_BASE_LAST

#define SEN15901_DRIVER_WIND_DIRECTION_PULL_UP_RESISTOR_OHMS    10000

#define SEN15901_DRIVER_WIND_SPEED_SAMPLING_TIME_SECONDS        1
#define SEN15901_DRIVER_WIND_DIRECTION_SAMPLING_PERIOD_SECONDS  10

#endif /* __SEN15901_DRIVER_FLAGS_H__ */
