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
void ADC_GetMcuVddMv(unsigned int* mcu_supply_voltage_mv);
void ADC_GetMcuTempDegrees(int* mcu_temp_degrees);

#endif /* PERIPHERALS_ADC_H */
