/*
 * adc.c
 *
 *  Created on: 5 may 2018
 *      Author: Ludo
 */

#include "adc.h"

#include "adc_reg.h"
#include "rcc_reg.h"
#include "tim.h"

/*** ADC local global variables ***/

unsigned int vcc_mv = 3300; // Default value = 3.3V.

/*** ADC functions ***/

/* INIT ADC1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void ADC1_Init(void) {

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

/* ENABLE ADC1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void ADC1_Enable(void) {

	/* Enable ADC peripheral */
	RCC -> APB2ENR |= (0b1 << 9); // ADCEN='1'.
	ADC1 -> CR |= (0b1 << 0); // ADEN='1'.
	while (((ADC1 -> ISR) & (0b1 << 0)) == 0); // Wait for ADC to be ready (ADRDY='1').
}

/* SWITCH ADC1 OFF.
 * @param:	None.
 * @return:	None.
 */
void ADC1_Disable(void) {

	/* Disable ADC1 peripheral */
	ADC1 -> CR &= ~(0b1 << 0); // ADEN='0'.
	RCC -> APB2ENR &= ~(0b1 << 9); // ADCEN='0'.
}

/* GET THE EFFECTIVE SUPPLY VOLTAGE THANKS TO THE INTERNAL VOLTAGE REFERENCE.
 * @param mcu_supply_voltage_mv:	Pointer to value that will contain MCU supply voltage in mV.
 * @return:							None.
 */
void ADC1_GetMcuSupplyVoltage(unsigned int* supply_voltage_mv) {

	/* Select ADC_IN17 input channel */
	ADC1 -> CHSELR = 0;
	ADC1 -> CHSELR |= (0b1 << 17);

	/* Set sampling time (see p.89 of STM32L031x4/6 datasheet) */
	ADC1 -> SMPR &= ~(0b111 << 0); // Reset bits 0-2.
	ADC1 -> SMPR |= (0b110 << 0); // Sampling time for internal voltage reference is 10탎 typical, 79.5*(1/ADCCLK) = 9.94탎 for SYSCLK = 16MHz;

	/* Wake-up internal voltage reference */
	ADC1 -> CCR |= (0b1 << 22); // VREFEN='1'.
	TIM22_WaitMilliseconds(10); // Wait al least 3ms (see p.55 of STM32L031x4/6 datasheet).

	/* Read raw supply voltage */
	ADC1 -> CR |= (0b1 << 2); // ADSTART='1'.
	while (((ADC1 -> ISR) & (0b1 << 2)) == 0); // Wait end of conversion ('EOC='1').
	unsigned int raw_supply_voltage_12bits = (ADC1 -> DR);

	/* Compute voltage according to MCU factory calibration (see p.301 of RM0377 datasheet) */
	vcc_mv = (VREFINT_VCC_CALIB_MV * VREFINT_CAL) / (raw_supply_voltage_12bits);
	(*supply_voltage_mv) = vcc_mv;

	/* Switch internal voltage reference off */
	ADC1 -> CCR &= ~(0b1 << 22); // VREFEN='0'.
}

/* GET THE DEVICE TEMPERATURE THANKS TO THE INTERNAL TEMPERATURE SENSOR.
 * @param mcu_temp_degrees:	Pointer to value that will contain MCU temperature in 캜.
 * @return:					None.
 */
void ADC1_GetMcuTemperature(int* temperature_degrees) {

	/* Select ADC_IN18 input channel*/
	ADC1 -> CHSELR = 0;
	ADC1 -> CHSELR |= (0b1 << 18);

	/* Set sampling time (see p.89 of STM32L031x4/6 datasheet) */
	ADC1 -> SMPR |= (0b111 << 0); // Sampling time for temperature sensor must be greater than 10탎, 160.5*(1/ADCCLK) = 20탎 for SYSCLK = 16MHz;

	/* Wake-up temperature sensor */
	ADC1 -> CCR |= (0b1 << 23); // TSEN='1'.
	TIM22_WaitMilliseconds(1); // Wait al least 10탎 (see p.89 of STM32L031x4/6 datasheet).

	/* Read raw temperature */
	ADC1 -> CR |= (0b1 << 2); // ADSTART='1'.
	while (((ADC1 -> ISR) & (0b1 << 2)) == 0); // Wait end of conversion ('EOC='1').
	int raw_temp_sensor_12bits = (ADC1 -> DR);

	/* Compute temperature according to MCU factory calibration (see p.301 and p.847 of RM0377 datasheet) */
	int raw_temp_calib_mv = (raw_temp_sensor_12bits * vcc_mv) / (TS_VCC_CALIB_MV) - TS_CAL1; // Equivalent raw measure for calibration power supply (VCC_CALIB).
	int temp_calib_degrees = raw_temp_calib_mv * ((int)(TS_CAL2_TEMP-TS_CAL1_TEMP));
	temp_calib_degrees = (temp_calib_degrees) / ((int)(TS_CAL2 - TS_CAL1));
	(*temperature_degrees) = temp_calib_degrees + TS_CAL1_TEMP;

	/* Switch temperature sensor off */
	ADC1 -> CCR &= ~(0b1 << 23); // TSEN='0'.
}
