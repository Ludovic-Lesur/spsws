/*
 * adc.h
 *
 *  Created on: 5 may 2018
 *      Author: Ludo
 */

#ifndef ADC_H
#define ADC_H

/*** ADC structures ***/

typedef enum {
	ADC_DATA_IDX_VMCU_MV = 0,
	ADC_DATA_IDX_LAST
} ADC_data_index_t;

typedef enum {
	ADC_SUCCESS = 0,
	ADC_ERROR_CALIBRATION,
	ADC_ERROR_INDEX,
	ADC_ERROR_TIMEOUT,
	ADC_ERROR_LAST
} ADC_status_t;

/*** ADC functions ***/

ADC_status_t ADC1_init(void);
void ADC1_disable(void);
ADC_status_t ADC1_perform_measurements(void);
ADC_status_t ADC1_get_data(ADC_data_index_t data_idx, unsigned int* data);
void ADC1_get_tmcu_comp2(signed char* tmcu_degrees);
void ADC1_get_tmcu_comp1(unsigned char* tmcu_degrees);

#define ADC_status_check(error_base) { if (adc_status != ADC_SUCCESS) { status = error_base + adc_status; goto errors; }}

#endif /* ADC_H */
