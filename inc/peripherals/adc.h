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
void ADC_GetMcuVddMv(unsigned int* mcu_vdd_mv);
void ADC_GetMcuTemperatureDegrees(int* mcu_temperature_degrees);
void ADC_GetHwtVoltageReferenceMv(unsigned int* hwt_voltage_reference);

#endif /* PERIPHERALS_ADC_H */
