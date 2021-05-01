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
void ADC1_Disable(void);
void ADC1_PerformAllMeasurements(void);
void ADC1_GetMcuVoltage(unsigned int* supply_voltage_mv);
void ADC1_GetMcuTemperatureComp2(signed char* mcu_temperature_degrees);
void ADC1_GetMcuTemperatureComp1(unsigned char* mcu_temperature_degrees);

#endif /* ADC_H */
