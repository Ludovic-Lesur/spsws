/*
 * max11136.h
 *
 *  Created on: 17 nov. 2018
 *      Author: Ludo
 */

#ifndef MAX11136_H
#define MAX11136_H

#include "mode.h"
#include "spi.h"

/*** MAX11136 structures ***/

typedef enum {
	MAX11136_SUCCESS = 0,
	MAX11136_ERROR_SPI,
	MAX11136_ERROR_TIMEOUT = (MAX11136_ERROR_SPI + SPI_ERROR_LAST),
	MAX11136_ERROR_CONVERSION,
	MAX11136_ERROR_DATA_INDEX,
	MAX11136_ERROR_LAST
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
MAX11136_status_t MAX11136_get_data(MAX11136_data_index_t data_idx, unsigned int* data);

#define MAX11136_status_check(error_base) { if (max11136_status != MAX11136_SUCCESS) { status = error_base + max11136_status; goto errors; }}

#endif /* MAX11136_H */
