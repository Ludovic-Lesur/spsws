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

#endif /* PERIPHERALS_ADC_H */
