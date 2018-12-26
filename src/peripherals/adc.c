/*
 * adc.c
 *
 *  Created on: 5 may 2018
 *      Author: Ludo
 */

#include "adc.h"

#include "adc_reg.h"
#include "max11136.h"
#include "rcc_reg.h"
#include "tim.h"

/*** ADC local structures ***/

typedef struct {
	signed char adc_mcu_temperature_degrees;
} ADC_Context;

/*** ADC local global variables ***/

ADC_Context adc_ctx;

/*** ADC functions ***/

/* INIT ADC1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void ADC1_Init(void) {

	/* Init context */
	adc_ctx.adc_mcu_temperature_degrees = 0;

	/* Enable peripheral clock */
	RCC -> APB2ENR |= (0b1 << 9); // ADCEN='1'.

	/* Disable ADC before configure it */
	ADC1 -> CR = 0; // ADEN='0'.

	/* Enable ADC voltage regulator */
	ADC1 -> CR |= (0b1 << 28);
	TIM22_WaitMilliseconds(5);

	/* ADC configuration */
	ADC1 -> CFGR2 &= ~(0b11 << 30); // Reset bits 30-31.
	ADC1 -> CFGR2 |= (0b01 << 30); // Use (PCLK2/2) as ADCCLK = SYSCLK/2 (see RCC_Init() function).
	ADC1 -> CFGR1 &= (0b1 << 13); // Single conversion mode.
	ADC1 -> CFGR1 &= ~(0b11 << 0); // Data rsolution = 12 bits (RES='00').

	/* ADC calibration */
	ADC1 -> CR |= (0b1 << 31); // ADCAL='1'.
	while (((ADC1 -> CR) & (0b1 << 31)) != 0); // Wait until calibration is done.

	/* Disable peripheral by default */
	RCC -> APB2ENR |= (0b1 << 9); // ADCEN='0'.
}

/* PERFORM INTERNAL ADC MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void ADC1_PerformMeasurements(void) {

	/* Enable ADC peripheral */
	RCC -> APB2ENR |= (0b1 << 9); // ADCEN='1'.
	ADC1 -> CR |= (0b1 << 0); // ADEN='1'.
	while (((ADC1 -> ISR) & (0b1 << 0)) == 0); // Wait for ADC to be ready (ADRDY='1').

	/* Select ADC_IN18 input channel*/
	ADC1 -> CHSELR = 0;
	ADC1 -> CHSELR |= (0b1 << 18);

	/* Set sampling time (see p.89 of STM32L031x4/6 datasheet) */
	ADC1 -> SMPR |= (0b111 << 0); // Sampling time for temperature sensor must be greater than 10µs, 160.5*(1/ADCCLK) = 20µs for SYSCLK = 16MHz;

	/* Wake-up temperature sensor */
	ADC1 -> CCR |= (0b1 << 23); // TSEN='1'.
	TIM22_WaitMilliseconds(1); // Wait al least 10µs (see p.89 of STM32L031x4/6 datasheet).

	/* Read raw temperature */
	ADC1 -> CR |= (0b1 << 2); // ADSTART='1'.
	while (((ADC1 -> ISR) & (0b1 << 2)) == 0); // Wait end of conversion ('EOC='1').
	int raw_temp_sensor_12bits = (ADC1 -> DR);

	/* Compute temperature according to MCU factory calibration (see p.301 and p.847 of RM0377 datasheet) */
	unsigned int vcc_mv = 0;
	MAX11136_GetSupplyVoltage(&vcc_mv);
	int raw_temp_calib_mv = (raw_temp_sensor_12bits * vcc_mv) / (TS_VCC_CALIB_MV) - TS_CAL1; // Equivalent raw measure for calibration power supply (VCC_CALIB).
	int temp_calib_degrees = raw_temp_calib_mv * ((int)(TS_CAL2_TEMP-TS_CAL1_TEMP));
	temp_calib_degrees = (temp_calib_degrees) / ((int)(TS_CAL2 - TS_CAL1));
	adc_ctx.adc_mcu_temperature_degrees = temp_calib_degrees + TS_CAL1_TEMP;

	/* Switch temperature sensor off */
	ADC1 -> CCR &= ~(0b1 << 23); // TSEN='0'.

	/* Disable ADC1 peripheral */
	ADC1 -> CR &= ~(0b1 << 0); // ADEN='0'.
	RCC -> APB2ENR &= ~(0b1 << 9); // ADCEN='0'.
}

/* GET THE DEVICE TEMPERATURE THANKS TO THE INTERNAL TEMPERATURE SENSOR.
 * @param mcu_temp_degrees:	Pointer to value that will contain MCU temperature in °C.
 * @return:					None.
 */
void ADC1_GetMcuTemperature(signed char* temperature_degrees) {

	/* Get result */
	(*temperature_degrees) = adc_ctx.adc_mcu_temperature_degrees;
}
