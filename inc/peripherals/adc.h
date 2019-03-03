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
void ADC1_PerformMeasurements(void);
void ADC1_GetMcuSupplyVoltage(unsigned int* supply_voltage_mv);
void ADC1_GetMcuTemperature(signed char* temperature_degrees);

#endif /* ADC_H */
