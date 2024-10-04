/*
 * max111xx_driver_flags.h
 *
 *  Created on: 01 sep. 2024
 *      Author: Ludo
 */

#ifndef __MAX111XX_DRIVER_FLAGS_H__
#define __MAX111XX_DRIVER_FLAGS_H__

#include "lptim.h"
#include "spi.h"

/*** MAX111xx driver compilation flags ***/

#define MAX111XX_DRIVER_GPIO_ERROR_BASE_LAST    0
#define MAX111XX_DRIVER_SPI_ERROR_BASE_LAST     SPI_ERROR_BASE_LAST
#define MAX111XX_DRIVER_DELAY_ERROR_BASE_LAST   LPTIM_ERROR_BASE_LAST

#define MAX111XX_DRIVER_NUMBER_OF_CHANNELS      8

#define MAX111XX_DRIVER_NUMBER_OF_BITS          12

#endif /* __MAX111XX_DRIVER_FLAGS_H__ */
