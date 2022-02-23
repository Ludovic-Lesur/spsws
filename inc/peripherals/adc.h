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
	ADC_DATA_IDX_MAX
} ADC_DataIndex;

/*** ADC functions ***/

void ADC1_Init(void);
void ADC1_Disable(void);
void ADC1_PerformMeasurements(void);
void ADC1_GetData(ADC_DataIndex adc_data_idx, unsigned int* data);
void ADC1_GetTmcuComp2(signed char* tmcu_degrees);
void ADC1_GetTmcuComp1(unsigned char* tmcu_degrees);

#endif /* ADC_H */
