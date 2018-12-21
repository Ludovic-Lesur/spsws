/*
 * adc.h
 *
 *  Created on: 5 may 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_ADC_H
#define PERIPHERALS_ADC_H

/*** ADC functions ***/

void ADC_Init(void);
void ADC_Off(void);
void ADC_GetMcuSupplyVoltage(unsigned int* supply_voltage_mv);
void ADC_GetMcuTemperature(int* temperature_degrees);
void ADC_GetHwtVoltageReferenceMv(unsigned int* hwt_voltage_reference);

#endif /* PERIPHERALS_ADC_H */
