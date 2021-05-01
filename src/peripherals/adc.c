/*
 * adc.c
 *
 *  Created on: 5 may 2018
 *      Author: Ludo
 */

#include "adc.h"

#include "adc_reg.h"
#include "filter.h"
#include "lptim.h"
#include "rcc_reg.h"

/*** ADC local macros ***/

#define ADC_TIMEOUT_COUNT					1000000
#define ADC_CHANNEL_VREFINT					17
#define ADC_CHANNEL_TEMPERATURE_SENSOR		18

#define ADC_MEDIAN_FILTER_LENGTH			9
#define ADC_CENTER_AVERGAE_LENGTH			3

/*** ADC local structures ***/

typedef struct {
	unsigned int adc_mcu_voltage_mv;
	unsigned char adc_mcu_temperature_degrees_comp1;
	signed char adc_mcu_temperature_degrees_comp2;
} ADC_Context;

/*** ADC local global variables ***/

static ADC_Context adc_ctx;

/*** ADC local functions ***/

/* PERFORM A SINGLE ADC CONVERSION.
 * @param adc_channel:			Channel to convert.
 * @param adc_result_12bits:	Pointer to int that will contain ADC raw result on 12 bits.
 * @return:						None.
 */
static void ADC1_SingleConversion(unsigned char adc_channel, unsigned int* adc_result_12bits) {
	// Select input channel.
	ADC1 -> CHSELR &= 0xFFF80000; // Reset all bits.
	ADC1 -> CHSELR |= (0b1 << adc_channel);
	// Read raw supply voltage.
	ADC1 -> CR |= (0b1 << 2); // ADSTART='1'.
	unsigned int loop_count = 0;
	while (((ADC1 -> ISR) & (0b1 << 2)) == 0) {
		// Wait end of conversion ('EOC='1') or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) return;
	}
	(*adc_result_12bits) = (ADC1 -> DR);
}

/* PERFORM SEVERAL CONVERSIONS FOLLOWED BY A MEDIAN FIILTER.
 * @param adc_channel:			Channel to convert.
 * @param adc_result_12bits:	Pointer to int that will contain ADC filtered result on 12 bits.
 * @return:						None.
 */
static void ADC1_FilteredConversion(unsigned char adc_channel, unsigned int* adc_result_12bits) {
	// Perform all conversions.
	unsigned int adc_sample_buf[ADC_MEDIAN_FILTER_LENGTH] = {0x00};
	unsigned char idx = 0;
	for (idx=0 ; idx<ADC_MEDIAN_FILTER_LENGTH ; idx++) {
		ADC1_SingleConversion(adc_channel, &(adc_sample_buf[idx]));
	}
	// Apply median filter.
	(*adc_result_12bits) = FILTER_ComputeMedianFilter(adc_sample_buf, ADC_MEDIAN_FILTER_LENGTH, ADC_CENTER_AVERGAE_LENGTH);
}

/* COMPUTE MCU SUPPLY VOLTAGE THANKS TO INTERNAL VOLTAGE REFERENCE.
 * @param:	None.
 * @return:	None.
 */
static void ADC1_ComputeMcuVoltage(void) {
	// Set sampling time (see p.89 of STM32L031x4/6 datasheet).
	ADC1 -> SMPR &= ~(0b111 << 0); // Reset bits 0-2.
	ADC1 -> SMPR |= (0b110 << 0); // Sampling time for internal voltage reference is 10us typical, 79.5*(1/ADCCLK) = 9.94us for ADCCLK = SYSCLK/2 = 8MHz;
	// Wake-up internal voltage reference.
	ADC1 -> CCR |= (0b1 << 22); // VREFEN='1'.
	LPTIM1_DelayMilliseconds(10, 0); // Wait al least 3ms (see p.55 of STM32L031x4/6 datasheet).
	// Read raw supply voltage.
	int raw_supply_voltage_12bits = 0;
	ADC1_FilteredConversion(ADC_CHANNEL_VREFINT, &raw_supply_voltage_12bits);
	// Compute temperature according to MCU factory calibration (see p.301 of RM0377 datasheet).
	adc_ctx.adc_mcu_voltage_mv = (VREFINT_VCC_CALIB_MV * VREFINT_CAL) / (raw_supply_voltage_12bits);
	// Switch internal voltage reference off.
	ADC1 -> CCR &= ~(0b1 << 22); // VREFEN='0'.
}

/* COMPUTE MCU TEMPERATURE THANKS TO INTERNAL VOLTAGE REFERENCE.
 * @param:	None.
 * @return:	None.
 */
static void ADC1_ComputeMcuTemperature(void) {
	// Set sampling time (see p.88 of STM32L031x4/6 datasheet).
	ADC1 -> SMPR |= (0b111 << 0); // Sampling time for temperature sensor must be greater than 10us, 160.5*(1/ADCCLK) = 20us for ADCCLK = SYSCLK/2 = 8MHz;
	// Wake-up VREFINT and temperature sensor.
	ADC1 -> CCR |= (0b11 << 22); // TSEN='1' and VREFEF='1'.
	LPTIM1_DelayMilliseconds(10, 0); // Wait internal reference stabilization (max 3ms).
	// Read raw temperature.
	int raw_temp_sensor_12bits = 0;
	ADC1_FilteredConversion(ADC_CHANNEL_TEMPERATURE_SENSOR, &raw_temp_sensor_12bits);
	// Compute temperature according to MCU factory calibration (see p.301 and p.847 of RM0377 datasheet).
	int raw_temp_calib_mv = (raw_temp_sensor_12bits * adc_ctx.adc_mcu_voltage_mv) / (TS_VCC_CALIB_MV) - TS_CAL1; // Equivalent raw measure for calibration power supply (VCC_CALIB).
	int temp_calib_degrees = raw_temp_calib_mv * ((int)(TS_CAL2_TEMP-TS_CAL1_TEMP));
	temp_calib_degrees = (temp_calib_degrees) / ((int)(TS_CAL2 - TS_CAL1));
	adc_ctx.adc_mcu_temperature_degrees_comp2 = temp_calib_degrees + TS_CAL1_TEMP;
	// Switch temperature sensor off.
	ADC1 -> CCR &= ~(0b1 << 23); // TSEN='0'.
	// Convert to 1-complement value.
	adc_ctx.adc_mcu_temperature_degrees_comp1 = 0;
	if (adc_ctx.adc_mcu_temperature_degrees_comp2 < 0) {
		adc_ctx.adc_mcu_temperature_degrees_comp1 |= 0x80;
		unsigned char temperature_abs = (-1) * (adc_ctx.adc_mcu_temperature_degrees_comp2);
		adc_ctx.adc_mcu_temperature_degrees_comp1 |= (temperature_abs & 0x7F);
	}
	else {
		adc_ctx.adc_mcu_temperature_degrees_comp1 = (adc_ctx.adc_mcu_temperature_degrees_comp2 & 0x7F);
	}
}

/*** ADC functions ***/

/* INIT ADC1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void ADC1_Init(void) {
	// Init context.
	adc_ctx.adc_mcu_voltage_mv = 0;
	adc_ctx.adc_mcu_temperature_degrees_comp2 = 0;
	adc_ctx.adc_mcu_temperature_degrees_comp1 = 0;
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 9); // ADCEN='1'.
	// Ensure ADC is disabled.
	if (((ADC1 -> CR) & (0b1 << 0)) != 0) {
		ADC1 -> CR |= (0b1 << 1); // ADDIS='1'.
	}
	// Enable ADC voltage regulator.
	ADC1 -> CR |= (0b1 << 28);
	LPTIM1_DelayMilliseconds(5, 0);
	// ADC configuration.
	ADC1 -> CFGR2 &= ~(0b11 << 30); // Reset bits 30-31.
	ADC1 -> CFGR2 |= (0b01 << 30); // Use (PCLK2/2) as ADCCLK = SYSCLK/2 (see RCC_Init() function).
	ADC1 -> CFGR1 &= (0b1 << 13); // Single conversion mode.
	ADC1 -> CFGR1 &= ~(0b11 << 0); // Data resolution = 12 bits (RES='00').
	ADC1 -> CCR &= 0xFC03FFFF; // No prescaler.
	ADC1 -> SMPR |= (0b111 << 0); // Maximum sampling time.
	// ADC calibration.
	ADC1 -> CR |= (0b1 << 31); // ADCAL='1'.
	unsigned int loop_count = 0;
	while ((((ADC1 -> CR) & (0b1 << 31)) != 0) && (((ADC1 -> ISR) & (0b1 << 11)) == 0)) {
		// Wait until calibration is done or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) break;
	}
	// Clear all flags.
	ADC1 -> ISR |= 0x0000089F;
}

/* DISABLE INTERNAL ADC PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void ADC1_Disable(void) {
	// Disable peripheral.
	if (((ADC1 -> CR) & (0b1 << 0)) != 0) {
		ADC1 -> CR |= (0b1 << 1); // ADDIS='1'.
	}
	// Clear all flags.
	ADC1 -> ISR |= 0x0000089F;
	// Disable peripheral clock.
	RCC -> APB2ENR &= ~(0b1 << 9); // ADCEN='0'.
}

/* PERFORM INTERNAL ADC MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void ADC1_PerformAllMeasurements(void) {
	// Enable ADC peripheral.
	ADC1 -> CR |= (0b1 << 0); // ADEN='1'.
	unsigned int loop_count = 0;
	while (((ADC1 -> ISR) & (0b1 << 0)) == 0) {
		// Wait for ADC to be ready (ADRDY='1') or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) return;
	}
	// Perform measurements.
	ADC1_ComputeMcuVoltage();
	ADC1_ComputeMcuTemperature();
	// Clear all flags.
	ADC1 -> ISR |= 0x0000089F; // Clear all flags.
	// Disable ADC peripheral.
	if (((ADC1 -> CR) & (0b1 << 0)) != 0) {
		ADC1 -> CR |= (0b1 << 1); // ADDIS='1'.
	}
}

/* GET MCU SUPPLY VOLTAGE.
 * @param mcu_voltage_mv:	Pointer to value that will contain MCU supply voltage in mV.
 * @return:					None.
 */
void ADC1_GetMcuVoltage(unsigned int* mcu_voltage_mv) {
	(*mcu_voltage_mv) = adc_ctx.adc_mcu_voltage_mv;
}

/* GET MCU TEMPERATURE.
 * @param mcu_temperature_degrees:	Pointer to signed value that will contain MCU temperature in degrees (2-complement).
 * @return:							None.
 */
void ADC1_GetMcuTemperatureComp2(signed char* mcu_temperature_degrees) {
	(*mcu_temperature_degrees) = adc_ctx.adc_mcu_temperature_degrees_comp2;
}

/* GET MCU TEMPERATURE.
 * @param mcu_temperature_degrees:	Pointer to unsigned value that will contain MCU temperature in degrees (1-complement).
 * @return:							None.
 */
void ADC1_GetMcuTemperatureComp1(unsigned char* mcu_temperature_degrees) {
	(*mcu_temperature_degrees) = adc_ctx.adc_mcu_temperature_degrees_comp1;
}
