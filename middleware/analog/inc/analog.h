/*
 * analog.h
 *
 *  Created on: 12 aug. 2024
 *      Author: Ludo
 */

#ifndef __ANALOG_H__
#define __ANALOG_H__

#include "adc.h"
#include "max11136.h"
#include "mode.h"
#include "types.h"

/*** ANALOG structures ***/

/*!******************************************************************
 * \enum ANALOG_status_t
 * \brief ANALOG driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	ANALOG_SUCCESS = 0,
	ANALOG_ERROR_NULL_PARAMETER,
	ANALOG_ERROR_CHANNEL,
	// Low level drivers errors.
	ANALOG_ERROR_BASE_ADC = 0x0100,
	ANALOG_ERROR_BASE_MAX11136 = (ANALOG_ERROR_BASE_ADC + ADC_ERROR_BASE_LAST),
	// Last base value.
	ANALOG_ERROR_BASE_LAST = (ANALOG_ERROR_BASE_MAX11136 + MAX11136_ERROR_BASE_LAST)
} ANALOG_status_t;

/*!******************************************************************
 * \enum ANALOG_index_t
 * \brief ANALOG data indexes.
 *******************************************************************/
typedef enum {
	ANALOG_CHANNEL_VMCU_MV = 0,
	ANALOG_CHANNEL_TMCU_DEGREES,
	ANALOG_CHANNEL_VPV_MV,
	ANALOG_CHANNEL_VCAP_MV,
	ANALOG_CHANNEL_LDR_PERCENT,
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
	ANALOG_CHANNEL_WIND_DIRECTION_RATIO,
#endif
	ANALOG_CHANNEL_LAST
} ANALOG_channel_t;

/*** ANALOG functions ***/

/*!******************************************************************
 * \fn ANALOG_status_t ANALOG_init(void)
 * \brief Init ANALOG driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
ANALOG_status_t ANALOG_init(void);

/*!******************************************************************
 * \fn ANALOG_status_t ANALOG_de_init(void)
 * \brief Release ANALOG driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
ANALOG_status_t ANALOG_de_init(void);

/*!******************************************************************
 * \fn ANALOG_status_t ANALOG_perform(void)
 * \brief Perform all analog measurements.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
ANALOG_status_t ANALOG_convert_channel(ANALOG_channel_t channel, int32_t* analog_data);

/*******************************************************************/
#define ANALOG_exit_error(base) { if (analog_status != ANALOG_SUCCESS) { status = (base + analog_status); goto errors; } }

/*******************************************************************/
#define ANALOG_stack_error(base) { if (analog_status != ANALOG_SUCCESS) { ERROR_stack_add(base + analog_status); } }

/*******************************************************************/
#define ANALOG_stack_exit_error(base, code) { if (analog_status != ANALOG_SUCCESS) { ERROR_stack_add(base + analog_status); status = code; goto errors; } }

#endif /* __ANALOG_H__ */
