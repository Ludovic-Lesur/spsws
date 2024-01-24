/*
 * adc.h
 *
 *  Created on: 05 may 2018
 *      Author: Ludo
 */

#ifndef __ADC_H__
#define __ADC_H__

#include "lptim.h"
#include "math.h"
#include "types.h"

/*** ADC macros ***/

#define ADC_INIT_DELAY_MS_REGULATOR	5
#define ADC_INIT_DELAY_MS_VREF_TS	10

#define ADC_INIT_DELAY_MS			(ADC_INIT_DELAY_MS_REGULATOR + ADC_INIT_DELAY_MS_VREF_TS)

/*** ADC structures ***/

/*!******************************************************************
 * \enum ADC_status_t
 * \brief ADC driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	ADC_SUCCESS = 0,
	ADC_ERROR_NULL_PARAMETER,
	ADC_ERROR_DISABLE_TIMEOUT,
	ADC_ERROR_CALIBRATION,
	ADC_ERROR_READY_TIMEOUT,
	ADC_ERROR_CHANNEL,
	ADC_ERROR_CONVERSION_TYPE,
	ADC_ERROR_CONVERSION_TIMEOUT,
	ADC_ERROR_DATA_INDEX,
	// Low level drivers errors.
	ADC_ERROR_BASE_LPTIM1 = 0x0100,
	ADC_ERROR_BASE_MATH = (ADC_ERROR_BASE_LPTIM1 + LPTIM_ERROR_BASE_LAST),
	// Last base value.
	ADC_ERROR_BASE_LAST = (ADC_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST)
} ADC_status_t;

/*!******************************************************************
 * \enum ADC_data_index_t
 * \brief ADC data indexes.
 *******************************************************************/
typedef enum {
	ADC_DATA_INDEX_VMCU_MV = 0,
	ADC_DATA_INDEX_LAST
} ADC_data_index_t;

/*** ADC functions ***/

/*!******************************************************************
 * \fn ADC_status_t ADC1_init(void)
 * \brief Init ADC peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
ADC_status_t ADC1_init(void);

/*!******************************************************************
 * \fn void ADC1_de_init(void)
 * \brief Release ADC peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
ADC_status_t ADC1_de_init(void);

/*!******************************************************************
 * \fn ADC_status_t ADC1_perform_measurements(void)
 * \brief Perform all ADC channels measurement.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
ADC_status_t ADC1_perform_measurements(void);

/*!******************************************************************
 * \fn ADC_status_t ADC1_get_data(ADC_data_index_t data_idx, uint32_t* data)
 * \brief Read ADC conversion data.
 * \param[in]  	data_idx: Data to read.
 * \param[out] 	data: Pointer to integer that will contain the result.
 * \retval		Function execution status.
 *******************************************************************/
ADC_status_t ADC1_get_data(ADC_data_index_t data_idx, uint32_t* data);

/*!******************************************************************
 * \fn ADC_status_t ADC1_get_tmcu(int8_t* tmcu_degrees)
 * \brief Read ADC MCU temperature.
 * \param[in]  	none
 * \param[out] 	tmcu_degrees: Pointer to signed byte that will contain MCU temperature in 2's complement format.
 * \retval		Function execution status.
 *******************************************************************/
ADC_status_t ADC1_get_tmcu(int8_t* tmcu_degrees);

/*******************************************************************/
#define ADC1_exit_error(error_base) { if (adc1_status != ADC_SUCCESS) { status = (error_base + adc1_status); goto errors; } }

/*******************************************************************/
#define ADC1_stack_error(void) { if (adc1_status != ADC_SUCCESS) { ERROR_stack_add(ERROR_BASE_ADC1 + adc1_status); } }

/*******************************************************************/
#define ADC1_stack_exit_error(error_code) { if (adc1_status != ADC_SUCCESS) { ERROR_stack_add(ERROR_BASE_ADC1 + adc1_status); status = error_code; goto errors; } }

#endif /* __ADC_H__ */
