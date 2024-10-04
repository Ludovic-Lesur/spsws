/*
 * dps310_driver_flags.h
 *
 *  Created on: 30 aug. 2024
 *      Author: Ludo
 */

#ifndef __DPS310_DRIVER_FLAGS_H__
#define __DPS310_DRIVER_FLAGS_H__

#include "i2c.h"
#include "lptim.h"

/*** DPS310 driver compilation flags ***/

#define DPS310_DRIVER_I2C_ERROR_BASE_LAST       I2C_ERROR_BASE_LAST
#define DPS310_DRIVER_DELAY_ERROR_BASE_LAST     LPTIM_ERROR_BASE_LAST

#endif /* __DPS310_DRIVER_FLAGS_H__ */
