/*
 * sx1232_driver_flags.h
 *
 *  Created on: 01 sep. 2024
 *      Author: Ludo
 */

#ifndef __SX1232_DRIVER_FLAGS_H__
#define __SX1232_DRIVER_FLAGS_H__

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "lptim.h"
#include "spi.h"

/*** SX1232 driver compilation flags ***/

#define SX1232_DRIVER_SPI_ERROR_BASE_LAST       SPI_ERROR_BASE_LAST
#define SX1232_DRIVER_DELAY_ERROR_BASE_LAST     LPTIM_ERROR_BASE_LAST

#define SX1232_DRIVER_FXOSC_HZ                  32000000

#define SX1232_DRIVER_TX_ENABLE
#ifdef SIGFOX_EP_BIDIRECTIONAL
#define SX1232_DRIVER_RX_ENABLE
#endif

#endif /* __SX1232_DRIVER_FLAGS_H__ */
