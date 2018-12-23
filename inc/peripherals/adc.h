/*
 * adc.h
 *
 *  Created on: 5 may 2018
 *      Author: Ludo
 */

#ifndef ADC_H
#define ADC_H

/*** ADC functions ***/

void ADC1_Init(void);
void ADC1_Enable(void);
void ADC1_Disable(void);
void ADC1_GetMcuSupplyVoltage(unsigned int* supply_voltage_mv);
void ADC1_GetMcuTemperature(int* temperature_degrees);
void ADC1_GetHwtVoltageReferenceMv(unsigned int* hwt_voltage_reference);

#endif /* ADC_H */
