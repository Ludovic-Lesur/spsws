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

/*!******************************************************************
 * \enum MAX11136_status_t
 * \brief MAX11136 driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	MAX11136_SUCCESS = 0,
	MAX11136_ERROR_NULL_PARAMETER,
	MAX11136_ERROR_REGISTER_ADDRESS,
	MAX11136_ERROR_TIMEOUT,
	MAX11136_ERROR_CONVERSION,
	MAX11136_ERROR_DATA_INDEX,
	// Low level drivers errors.
#ifdef HW1_0
	MAX11136_ERROR_BASE_SPI1 = 0x0100,
	MAX11136_ERROR_BASE_LPTIM1 = (MAX11136_ERROR_BASE_SPI1 + SPI_ERROR_BASE_LAST),
#endif
#ifdef HW2_0
	MAX11136_ERROR_BASE_SPI2 = 0x0100,
	MAX11136_ERROR_BASE_LPTIM1 = (MAX11136_ERROR_BASE_SPI2 + SPI_ERROR_BASE_LAST),
#endif
	// Last base value.
	MAX11136_ERROR_BASE_LAST = (MAX11136_ERROR_BASE_LPTIM1 + LPTIM_ERROR_BASE_LAST)
} MAX11136_status_t;

/*!******************************************************************
 * \enum MAX11136_data_index_t
 * \brief MAX11136 data indexes.
 *******************************************************************/
typedef enum {
	MAX11136_DATA_INDEX_VSRC_MV,
	MAX11136_DATA_INDEX_VCAP_MV,
	MAX11136_DATA_INDEX_LDR_PERCENT,
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
	MAX11136_DATA_INDEX_WIND_DIRECTION_RATIO,
#endif
	MAX11136_DATA_INDEX_LAST
} MAX11136_data_index_t;

/*** MAX11136 functions ***/

/*!******************************************************************
 * \fn void MAX11136_init(void)
 * \brief Init MAX11136 driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void MAX11136_init(void);

/*!******************************************************************
 * \fn void MAX11136_de_init_init(void)
 * \brief Release MAX11136 driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void MAX11136_de_init(void);

/*!******************************************************************
 * \fn MAX11136_status_t MAX11136_perform_measurements(void)
 * \brief Perform analog measurements.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MAX11136_status_t MAX11136_perform_measurements(void);

/*!******************************************************************
 * \fn MAX11136_status_t MAX11136_get_data(MAX11136_data_index_t data_idx, uint32_t* data)
 * \brief Read MAX11136 conversion data.
 * \param[in]  	data_idx: Data to read.
 * \param[out] 	data: Pointer to integer that will contain the result.
 * \retval		Function execution status.
 *******************************************************************/
MAX11136_status_t MAX11136_get_data(MAX11136_data_index_t data_idx, uint32_t* data);

/*******************************************************************/
#define MAX11136_exit_error(error_base) { if (max11136_status != MAX11136_SUCCESS) { status = (error_base + max11136_status); goto errors; } }

/*******************************************************************/
#define MAX11136_stack_error(void) { if (max11136_status != MAX11136_SUCCESS) { ERROR_stack_add(ERROR_BASE_MAX11136 + max11136_status); } }

/*******************************************************************/
#define MAX11136_stack_exit_error(error_code) { if (max11136_status != MAX11136_SUCCESS) { ERROR_stack_add(ERROR_BASE_MAX11136 + max11136_status); status = error_code; goto errors; } }

#endif /* __MAX11136_H__ */
