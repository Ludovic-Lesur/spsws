/*
 * max11136.h
 *
 *  Created on: 17 nov. 2018
 *      Author: Ludo
 */

#ifndef __MAX11136_H__
#define __MAX11136_H__

#include "lptim.h"
#include "mode.h"
#include "spi.h"
#include "types.h"

/*** MAX11136 structures ***/

typedef enum {
	MAX11136_SUCCESS = 0,
	MAX11136_ERROR_TIMEOUT,
	MAX11136_ERROR_CONVERSION,
	MAX11136_ERROR_DATA_INDEX,
	MAX11136_ERROR_BASE_SPI = 0x0100,
	MAX11136_ERROR_BASE_LPTIM = (MAX11136_ERROR_BASE_SPI + SPI_ERROR_BASE_LAST),
	MAX11136_ERROR_BASE_LAST = (MAX11136_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST)
} MAX11136_status_t;

typedef enum {
	MAX11136_DATA_INDEX_VSRC_MV,
	MAX11136_DATA_INDEX_VCAP_MV,
	MAX11136_DATA_INDEX_LDR_PERCENT,
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
	MAX11136_DATA_WIND_DIRECTION_RATIO,
#endif
	MAX11136_DATA_INDEX_LAST
} MAX11136_data_index_t;

/*** MAX11136 functions ***/

void MAX11136_init(void);
void MAX11136_disable(void);
MAX11136_status_t MAX11136_perform_measurements(void);
MAX11136_status_t MAX11136_get_data(MAX11136_data_index_t data_idx, uint32_t* data);

#define MAX11136_status_check(error_base) { if (max11136_status != MAX11136_SUCCESS) { status = error_base + max11136_status; goto errors; }}
#define MAX11136_error_check() { ERROR_status_check(max11136_status, MAX11136_SUCCESS, ERROR_BASE_MAX11136); }
#define MAX11136_error_check_print() { ERROR_status_check_print(max11136_status, MAX11136_SUCCESS, ERROR_BASE_MAX11136); }

#endif /* __MAX11136_H__ */
