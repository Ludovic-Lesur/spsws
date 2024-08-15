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

/*** MAX11136 macros ***/

#define MAX11136_FULL_SCALE		4095

/*** MAX11136 structures ***/

/*!******************************************************************
 * \enum MAX11136_status_t
 * \brief MAX11136 driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	MAX11136_SUCCESS = 0,
	MAX11136_ERROR_NULL_PARAMETER,
	MAX11136_ERROR_CHANNEL,
	MAX11136_ERROR_REGISTER_ADDRESS,
	MAX11136_ERROR_TIMEOUT,
	MAX11136_ERROR_CONVERSION,
	MAX11136_ERROR_OUTPUT_CHANNEL,
	// Low level drivers errors.
	MAX11136_ERROR_BASE_SPI = 0x0100,
	MAX11136_ERROR_BASE_LPTIM = (MAX11136_ERROR_BASE_SPI + SPI_ERROR_BASE_LAST),
	// Last base value.
	MAX11136_ERROR_BASE_LAST = (MAX11136_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST)
} MAX11136_status_t;

/*!******************************************************************
 * \enum MAX11136_channel_t
 * \brief MAX11136 channels list.
 *******************************************************************/
typedef enum {
	MAX11136_CHANNEL_AIN0 = 0,
	MAX11136_CHANNEL_AIN1,
	MAX11136_CHANNEL_AIN2,
	MAX11136_CHANNEL_AIN3,
	MAX11136_CHANNEL_AIN4,
	MAX11136_CHANNEL_AIN5,
	MAX11136_CHANNEL_AIN6,
	MAX11136_CHANNEL_AIN7,
	MAX11136_CHANNEL_LAST
} MAX11136_channel_t;

/*** MAX11136 functions ***/

/*!******************************************************************
 * \fn MAX11136_status_t MAX11136_init(void)
 * \brief Init MAX11136 driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MAX11136_status_t MAX11136_init(void);

/*!******************************************************************
 * \fn MAX11136_status_t MAX11136_de_init_init(void)
 * \brief Release MAX11136 driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MAX11136_status_t MAX11136_de_init(void);

/*!******************************************************************
 * \fn MAX11136_status_t MAX11136_perform_measurements(void)
 * \brief Perform analog measurements.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MAX11136_status_t MAX11136_convert_channel(MAX11136_channel_t channel, uint16_t* adc_data_12bits);

/*******************************************************************/
#define MAX11136_exit_error(error_base) { if (max11136_status != MAX11136_SUCCESS) { status = (error_base + max11136_status); goto errors; } }

/*******************************************************************/
#define MAX11136_stack_error(void) { if (max11136_status != MAX11136_SUCCESS) { ERROR_stack_add(ERROR_BASE_MAX11136 + max11136_status); } }

/*******************************************************************/
#define MAX11136_stack_exit_error(error_code) { if (max11136_status != MAX11136_SUCCESS) { ERROR_stack_add(ERROR_BASE_MAX11136 + max11136_status); status = error_code; goto errors; } }

#endif /* __MAX11136_H__ */
